// Stand-in for CASSIE's Stroke MonoBehaviour: keeps the members FinalStroke uses, without rendering.
using UnityEngine;

public abstract class Stroke : MonoBehaviour
{
    protected float SubdivisionsPerUnit = 100f;
    protected virtual void Awake() { }
    public virtual void RenderAsLine(float scale) { }
    protected void RenderPoints(Vector3[] points, float scale) { }
    public virtual void Destroy() { }
}
