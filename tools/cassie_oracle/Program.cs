// Replays a CASSIE session through CASSIE's own C# graph (Graph, Node, Segment, Cycle,
// CycleDetection, FinalStroke, BezierCurve, LineCurve), driven the way interactor-curvenet's
// session_replay drives the C++ port, so the two can be compared event by event.
//   dotnet run -- <session.json> [--control-no-mirror-tap]
// --control-no-mirror-tap skips the mirrored tap, a planted defect the comparison must catch.
// Prints "ok ..." then one "state<TAB>a,b;c,d" line per stroke or tap event (the cycles that
// event added), then "FINAL<TAB>" and the live cycles.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using Curve;
using Newtonsoft.Json.Linq;
using UnityEngine;
using VRSketch;

class Logged
{
    public int id;
    public bool foundByAlgo;
    public List<int> strokes;
}

class Oracle
{
    Graph graph = new Graph();
    VRSketch.Plane plane;
    List<FinalStroke> strokes = new List<FinalStroke>();
    Dictionary<int, FinalStroke> mirrored = new Dictionary<int, FinalStroke>();
    List<Logged> pending = new List<Logged>();
    Dictionary<int, ICycle> alive = new Dictionary<int, ICycle>();
    int fresh = 1000000;
    public int unresolved, tapMisses, exceptions, missingDeletes;
    public static bool NoMirrorTap;
    static readonly FieldInfo CyclesField = typeof(Graph).GetField("_cycles", BindingFlags.NonPublic | BindingFlags.Instance);

    public Oracle(bool surfacing, Vector3 mirrorPoint, Vector3 mirrorNormal)
    {
        graph.Init(surfacing);
        plane = new VRSketch.Plane(mirrorNormal, mirrorPoint);
    }

    public List<List<int>> Cycles()
    {
        var set = (System.Collections.IEnumerable)CyclesField.GetValue(graph);
        var outp = new List<List<int>>();
        foreach (object o in set)
            outp.Add(((ICycle)o).Serialize().strokesID.Distinct().OrderBy(x => x).ToList());
        outp.Sort(Cmp);
        return outp;
    }

    public static int Cmp(List<int> a, List<int> b)
    {
        for (int i = 0; i < Math.Min(a.Count, b.Count); i++)
            if (a[i] != b[i]) return a[i].CompareTo(b[i]);
        return a.Count.CompareTo(b.Count);
    }

    static string Key(List<int> ids) => string.Join(",", ids.OrderBy(x => x));

    void Sink(List<ICycle> add, List<ICycle> remove)
    {
        foreach (ICycle c in remove) alive.Remove(c.GetPatchID());
        foreach (ICycle c in add)
        {
            SerializablePatch sp = c.Serialize();
            string k = Key(sp.strokesID);
            int pid = -1;
            for (int i = 0; i < pending.Count; i++)
            {
                if (Key(pending[i].strokes) == k && pending[i].foundByAlgo == sp.foundByAlgo)
                {
                    pid = pending[i].id;
                    pending.RemoveAt(i);
                    break;
                }
            }
            if (pid < 0) pid = fresh++;
            c.AssociateWithPatch(pid);
            alive[pid] = c;
        }
    }

    void GraphUpdate()
    {
        graph.TryFindAllCycles();
        graph.Update(out List<ICycle> add, out List<ICycle> remove);
        Sink(add, remove);
    }

    public void SetPending(List<Logged> p) { pending = new List<Logged>(p); }

    FinalStroke NewStroke(int id, Curve.Curve c, bool closed)
    {
        var fs = new FinalStroke();
        fs.Init(graph);
        fs.SetID(id);
        fs.SetCurve(c, closed);
        return fs;
    }

    static Curve.Curve MakeCurve(Vector3[] ctrl) =>
        ctrl.Length == 2 ? (Curve.Curve)new LineCurve(ctrl[0], ctrl[1], 1f, 1f) : new BezierCurve(ctrl);

    // Float64 Bernstein samples to shortlist strokes, as the port's ResolveStroke does.
    static IEnumerable<double[]> Dense(Curve.Curve c, int per)
    {
        List<Vector3> P = c.GetControlPoints();
        bool line = c is LineCurve;
        int nb = line ? 1 : (P.Count - 1) / 3;
        double step = 1.0 / (per - 1);
        for (int b = 0; b < nb; b++)
            for (int i = 0; i < per; i++)
            {
                double u = i == per - 1 ? 1.0 : i * step, w = 1.0 - u;
                var r = new double[3];
                for (int k = 0; k < 3; k++)
                {
                    if (line) r[k] = w * P[0][k] + u * P[1][k];
                    else
                    {
                        Vector3 p0 = P[3 * b], p1 = P[3 * b + 1], p2 = P[3 * b + 2], p3 = P[3 * b + 3];
                        r[k] = w * w * w * p0[k] + 3.0 * w * w * u * p1[k] + 3.0 * w * u * u * p2[k] + u * u * u * p3[k];
                    }
                }
                yield return r;
            }
    }

    FinalStroke Resolve(Vector3 pos, out PointOnCurve old, out float dist)
    {
        var best = new List<(double, int)>();
        foreach (FinalStroke st in strokes)
        {
            double m = double.PositiveInfinity;
            foreach (double[] q in Dense(st.Curve, 96))
            {
                double dx = q[0] - pos.x, dy = q[1] - pos.y, dz = q[2] - pos.z;
                m = Math.Min(m, Math.Sqrt(dx * dx + dy * dy + dz * dz));
            }
            best.Add((m, st.ID));
        }
        old = default;
        dist = 0f;
        if (best.Count == 0) return null;
        best.Sort();
        FinalStroke win = null;
        foreach (var b in best)
        {
            if (!(b.Item1 < best[0].Item1 + 2e-3)) continue;
            FinalStroke st = strokes.First(s => s.ID == b.Item2);
            PointOnCurve poc = st.Curve.Project(pos);
            float d = Vector3.Distance(poc.Position, pos);
            if (win == null || d < dist || (d == dist && st.ID < win.ID)) { win = st; dist = d; old = poc; }
        }
        return win;
    }

    PointOnCurve MirrorPos(PointOnCurve c) => new PointOnCurve(c.t, plane.Mirror(c.Position), c.Tangent, c.LocalNormal);

    public void AddStroke(int id, Vector3[] ctrl, List<(Vector3 pos, bool inter)> cons, bool closed, bool mirroring, float scale)
    {
        float sd = 0.02f / scale, snap = sd, merge = sd * 0.5f, prox = sd * 2f;
        Curve.Curve nc = MakeCurve(ctrl);
        bool onMirror = ctrl.All(p => Math.Abs(Vector3.Dot(plane.n, p - plane.p0)) < 1e-6f);
        var items = new List<(PointOnCurve nd, Vector3 pos, bool inter)>();
        foreach (var c in cons)
        {
            PointOnCurve nd;
            if (nc is LineCurve)
            {
                nd = nc.Project(c.pos);
                if (Vector3.Distance(nd.Position, c.pos) >= prox * 0.1f) continue;
            }
            else
            {
                int nbz = (ctrl.Length - 1) / 3, bi = 0;
                float ba = float.PositiveInfinity;
                for (int i = 0; i <= nbz; i++)
                {
                    float d = Vector3.Distance(ctrl[i * 3], c.pos);
                    if (d < ba) { ba = d; bi = i; }
                }
                nd = ((BezierCurve)nc).GetPointOnCurve(bi);
            }
            items.Add((nd, c.pos, c.inter));
        }
        if (nc is LineCurve) items = items.OrderBy(x => x.nd.t).ToList();
        var inters = new List<(FinalStroke old, PointOnCurve od, PointOnCurve nd)>();
        var seams = new List<PointOnCurve>();
        foreach (var it in items)
        {
            if (it.inter)
            {
                FinalStroke b = Resolve(it.pos, out PointOnCurve od, out float dist);
                if (b == null || dist > 1e-3f) { unresolved++; continue; }
                inters.Add((b, od, it.nd));
            }
            else if (Math.Abs((double)Vector3.Dot(plane.n, it.pos - plane.p0)) < 1e-5)
                seams.Add(it.nd);
        }
        try
        {
            FinalStroke fs = NewStroke(id, nc, closed);
            foreach (var i in inters)
            {
                INode node = i.old.AddIntersectionOldStroke(i.od, snap);
                fs.AddIntersectionNewStroke(node, i.nd, merge);
            }
            if (mirroring)
            {
                if (onMirror) mirrored[fs.ID] = fs;
                else
                {
                    Vector3[] mc = ctrl.Select(p => plane.Mirror(p)).ToArray();
                    FinalStroke ms = NewStroke(id + 1, MakeCurve(mc), closed);
                    mirrored[fs.ID] = ms;
                    mirrored[ms.ID] = fs;
                    foreach (var i in inters)
                    {
                        if (mirrored.TryGetValue(i.old.ID, out FinalStroke mo))
                        {
                            INode node = mo.AddIntersectionOldStroke(MirrorPos(i.od), snap);
                            ms.AddIntersectionNewStroke(node, MirrorPos(i.nd), merge);
                        }
                    }
                    foreach (PointOnCurve seam in seams)
                    {
                        INode node = fs.AddIntersectionOldStroke(seam, snap);
                        ms.AddIntersectionNewStroke(node, seam, merge);
                    }
                    strokes.Add(ms);
                    GraphUpdate();
                }
            }
            strokes.Add(fs);
            GraphUpdate();
        }
        catch (Exception e) { exceptions++; Console.Error.WriteLine("stroke " + id + ": " + e.Message); }
    }

    public void DeleteStroke(int id, bool mirroring)
    {
        FinalStroke s = strokes.FirstOrDefault(x => x.ID == id);
        if (s == null) return;
        try
        {
            if (mirrored.TryGetValue(s.ID, out FinalStroke ms))
            {
                mirrored.Remove(s.ID);
                mirrored.Remove(ms.ID);
                if (mirroring && ms.ID != s.ID) { strokes.Remove(ms); ms.Destroy(); }
            }
            strokes.Remove(s);
            s.Destroy();
            GraphUpdate();
        }
        catch (Exception e) { exceptions++; Console.Error.WriteLine("delete " + id + ": " + e.Message); }
    }

    // DrawingCanvas.TryAddPatchAt with the tap already in canvas space.
    public void Tap(List<Logged> group, Vector3 pos, bool mirroring)
    {
        pending = new List<Logged>(group);
        try
        {
            bool nm = false;
            bool ok = graph.TryFindCycleAt(pos, nm);
            if (!ok) { nm = true; ok = graph.TryFindCycleAt(pos, nm); }
            if (ok)
            {
                if (mirroring && !NoMirrorTap) graph.TryFindCycleAt(plane.Mirror(pos), nm);
                GraphUpdate();
            }
            else tapMisses++;
        }
        catch (Exception e) { exceptions++; Console.Error.WriteLine("tap: " + e.Message); }
    }

    public void DeletePatch(int pid)
    {
        if (!alive.ContainsKey(pid)) { missingDeletes++; return; }
        graph.ManualDeletePatch(pid);
        alive.Remove(pid);
    }
}

static class Program
{
    static Vector3 P(JToken t) => t == null ? Vector3.zero : new Vector3((float)(double)t["x"], (float)(double)t["y"], (float)(double)t["z"]);
    // Missing fields read as false, as the C++ replay's JSON reader does (older logs such as vacuum lack them).
    static bool B(JToken t) => t != null && t.Type == JTokenType.Boolean && (bool)t;

    static bool CanvasPoint(JToken st, out Vector3 r)
    {
        r = default;
        JToken h = st["primaryHandPos"], t = st["canvasPos"], q = st["canvasRot"];
        if (h == null || t == null || q == null || st["canvasScale"] == null) return false;
        double vx = (double)h["x"] - (double)t["x"], vy = (double)h["y"] - (double)t["y"], vz = (double)h["z"] - (double)t["z"];
        double qx = -(double)q["x"], qy = -(double)q["y"], qz = -(double)q["z"], qw = (double)q["w"];
        double tx = 2.0 * (qy * vz - qz * vy), ty = 2.0 * (qz * vx - qx * vz), tz = 2.0 * (qx * vy - qy * vx);
        double s = (double)st["canvasScale"];
        r = new Vector3((float)((vx + qw * tx + (qy * tz - qz * ty)) / s), (float)((vy + qw * ty + (qz * tx - qx * tz)) / s),
                (float)((vz + qw * tz + (qx * ty - qy * tx)) / s));
        return true;
    }

    static string Fmt(List<List<int>> cs) => string.Join(";", cs.Select(c => string.Join(",", c)));

    static int Main(string[] args)
    {
        Oracle.NoMirrorTap = args.Contains("--control-no-mirror-tap");
        JObject root = JObject.Parse(File.ReadAllText(args.First(a => !a.StartsWith("--"))));
        int model = (int)root["sketchModel"], mode = (int)root["interactionMode"];
        double[][] mp = { new[] { -0.125, 0.125, 0.125 }, new[] { 0.125, 0.125, 0.0 }, new[] { 0.125, 0.125, 0.0 }, new[] { 0.0, 0.125, 0.125 } };
        double[][] mn = { new[] { 1.0, 0, 0 }, new[] { 0, 0, 1.0 }, new[] { 0, 0, 1.0 }, new[] { 1.0, 0, 0 } };
        double[] origin = mode == 2 ? new[] { 0.25, 1.0, 0.75 } : new[] { 0.0, 1.0, 0.75 };
        var mirrorPoint = new Vector3((float)(mp[model][0] + origin[0]), (float)(mp[model][1] + origin[1]), (float)(mp[model][2] + origin[2]));
        var mirrorNormal = new Vector3((float)mn[model][0], (float)mn[model][1], (float)mn[model][2]);
        var o = new Oracle((int)root["sketchSystem"] == 2, mirrorPoint, mirrorNormal);

        var byId = root["allSketchedStrokes"].ToDictionary(s => (int)s["id"]);
        var patches = root["allCreatedPatches"].ToDictionary(p => (int)p["id"],
                p => new Logged { id = (int)p["id"], foundByAlgo = (bool)p["foundByAlgo"], strokes = p["strokesID"].Select(x => (int)x).ToList() });
        var states = (JArray)root["systemStates"];
        var pendingLog = new List<Logged>();
        var lines = new List<string>();
        int i = 0;
        while (i < states.Count)
        {
            JToken st = states[i];
            int type = (int)st["interactionType"], id = (int)st["elementID"];
            bool mirroring = B(st["mirroring"]);
            int next = i + 1;
            List<List<int>> before = null;
            if (type == 3 && patches[id].foundByAlgo) pendingLog.Add(patches[id]);
            else if (type == 3)
            {
                var grp = new List<Logged> { patches[id] };
                while (next < states.Count && (int)states[next]["interactionType"] == 3 && !patches[(int)states[next]["elementID"]].foundByAlgo
                        && (double)states[next]["time"] == (double)st["time"])
                {
                    grp.Add(patches[(int)states[next]["elementID"]]);
                    next++;
                }
                if (!CanvasPoint(st, out Vector3 tap)) { Console.WriteLine("error tapped patch " + id + " has no canvas transform"); return 1; }
                before = o.Cycles();
                o.Tap(grp, tap, mirroring);
            }
            else if (type == 4) o.DeletePatch(id);
            else if (type == 1 || type == 2)
            {
                if (type == 1) before = o.Cycles();
                o.SetPending(pendingLog);
                pendingLog.Clear();
                if (type == 1)
                {
                    JToken s = byId[id];
                    Vector3[] ctrl = s["ctrlPts"].Select(P).ToArray();
                    var rec = s["appliedPositionConstraints"].ToList();
                    if (ctrl.Length == 2) rec.AddRange(s["rejectedPositionConstraints"]);
                    var cons = rec.Select(c => (P(c["position"]), B(c["isIntersection"]))).ToList();
                    o.AddStroke(id, ctrl, cons, B(s["closedLoop"]), mirroring, (float)(double)st["canvasScale"]);
                }
                else o.DeleteStroke(id, mirroring);
            }
            if (before != null)
            {
                var after = o.Cycles();
                var added = after.Where(c => !before.Any(b => b.SequenceEqual(c))).ToList();
                added.Sort(Oracle.Cmp);
                lines.Add(i + "\t" + Fmt(added));
            }
            i = next;
        }
        Console.WriteLine($"ok unresolved={o.unresolved} tap_misses={o.tapMisses} missing_patch_deletes={o.missingDeletes} exceptions={o.exceptions}");
        foreach (string l in lines) Console.WriteLine(l);
        Console.WriteLine("FINAL\t" + Fmt(o.Cycles()));
        return 0;
    }
}
