using UnityEngine;
using System.Collections;
using Curve;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using Stopwatch = System.Diagnostics.Stopwatch;
using System.IO;
using MathNet.Numerics.LinearAlgebra.Single;

namespace VRSketch
{

    public class ConstraintCandidate
    {
        public Constraint constraint { get; }
        public float t { get; }
        public float likelihood { get; }
        public float score { get; }
        public bool active;
        public bool closeToEndpoint { get; }
        public bool alignedTangents { get; }

        public ConstraintCandidate(Constraint c, float t, float likelihood, float score, bool closeToEndpoint, bool alignedTangents)
        {
            constraint = c;
            active = true;
            this.t = t;
            this.likelihood = likelihood;
            this.score = score;
            this.closeToEndpoint = closeToEndpoint;
            this.alignedTangents = alignedTangents;
        }
    }

}
