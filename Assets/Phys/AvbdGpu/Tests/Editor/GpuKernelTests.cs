using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Kernel-level checks: scan, CSR lists, colouring validity, determinism.</summary>
    public class GpuKernelTests
    {
        [Test]
        public void ScanMatchesCpu()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            var kernels = new AvbdGpuKernels();
            var rng = new Unity.Mathematics.Random(7u);
            foreach (int n in new[] { 1, 63, 1024, 1025, 5000, 131072 })
            {
                var input = new uint[n];
                for (int i = 0; i < n; i++) input[i] = rng.NextUInt(0, 5);
                using var src = new GraphicsBuffer(GraphicsBuffer.Target.Structured, n, 4);
                using var dst = new GraphicsBuffer(GraphicsBuffer.Target.Structured, n + 1, 4);
                using var sums = new GraphicsBuffer(GraphicsBuffer.Target.Structured, 1024, 4);
                src.SetData(input);
                var cs = kernels.Scan;
                foreach (int k in new[] { kernels.ScanBlock, kernels.ScanTop, kernels.ScanAdd })
                {
                    cs.SetBuffer(k, "_ScanIn", src);
                    cs.SetBuffer(k, "_ScanOut", dst);
                    cs.SetBuffer(k, "_BlockSums", sums);
                }
                cs.SetInt("_ScanN", n);
                int groups = (n + 1023) / 1024;
                cs.Dispatch(kernels.ScanBlock, groups, 1, 1);
                cs.Dispatch(kernels.ScanTop, 1, 1, 1);
                cs.Dispatch(kernels.ScanAdd, groups, 1, 1);
                var output = new uint[n + 1];
                dst.GetData(output);
                uint sum = 0;
                for (int i = 0; i < n; i++)
                {
                    Assert.AreEqual(sum, output[i], $"n={n} element {i}");
                    sum += input[i];
                }
                Assert.AreEqual(sum, output[n], $"n={n} total");
            }
        }

        [Test]
        public void JointSceneStepsAreFinite()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Rope);
            for (int i = 0; i < 60; i++) world.Step();
            GpuTestUtil.AssertFinite(world);
            var stats = world.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, "no capacity overflow");
            Assert.AreEqual(19, stats.Constraints - stats.Manifolds, "19 joints in the constraint index space");
        }

        [Test]
        public void ColoringIsValidOnPyramid()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Pyramid);
            world.SetActiveColors(16);
            for (int i = 0; i < 120; i++) world.Step();
            var stats = world.GetStatsSync();
            Assert.Greater(stats.Manifolds, 100, "the pyramid has contacts");

            // Read the final colours and the CSR lists; no two dynamic neighbours may share a proper colour.
            int n = world.BodyCount;
            var color = new uint[n]; world.Buffers.BodyColor.GetData(color, 0, 0, n);
            var start = new uint[n + 1]; world.Buffers.BodyConsStart.GetData(start, 0, 0, n + 1);
            var list = new uint[math.max((int)start[n], 1)]; if (start[n] > 0) world.Buffers.BodyConsList.GetData(list, 0, 0, (int)start[n]);
            var manifolds = world.GetManifoldsSync(out _);
            int edges = 0;
            for (int body = 0; body < n; body++)
            {
                if (world.GetBodyDef(body).Mass <= 0) continue;
                Assert.Less(color[body], (uint)AvbdGpuConstants.MaxColors + 1, $"body {body} colour {color[body]}");
                for (uint k = start[body]; k < start[body + 1]; k++)
                {
                    uint r = list[k];
                    if (ConsRef.Type(r) != ConsRef.Manifold) continue;
                    var m = manifolds[ConsRef.Index(r)];
                    int other = (int)(m.BodyA == (uint)body ? m.BodyB : m.BodyA);
                    if (world.GetBodyDef(other).Mass <= 0) continue;
                    edges++;
                    if (color[body] < (uint)stats.ActiveColors && color[other] < (uint)stats.ActiveColors)
                        Assert.AreNotEqual(color[body], color[other], $"bodies {body} and {other} share colour {color[body]}");
                }
            }
            Assert.Greater(edges, 100);
            Debug.Log($"pyramid: {stats.Manifolds} manifolds, {stats.Contacts} contacts, colours used {stats.ColorsUsed}, overflow bodies {stats.OverflowBodies}, active {stats.ActiveColors}");
            Assert.AreEqual(0, stats.OverflowBodies, "the pyramid colours without overflow");
        }

        [Test]
        public void CsrListsContainEveryConstraintTwice()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Stack);
            for (int i = 0; i < 150; i++) world.Step();
            var stats = world.GetStatsSync();
            int n = world.BodyCount;
            var start = new uint[n + 1]; world.Buffers.BodyConsStart.GetData(start, 0, 0, n + 1);
            // ground (static) has no list; each of the 10 manifolds (ground-cube, cube-cube) appears once per dynamic endpoint
            Assert.AreEqual(0u, start[1] - start[0], "static ground has no constraints");
            int expected = 0;
            var manifolds = world.GetManifoldsSync(out _);
            for (int m = 0; m < stats.Manifolds; m++)
            {
                if (world.GetBodyDef((int)manifolds[m].BodyA).Mass > 0) expected++;
                if (world.GetBodyDef((int)manifolds[m].BodyB).Mass > 0) expected++;
            }
            Assert.AreEqual(expected, (int)start[n], "total CSR entries");
            Assert.GreaterOrEqual(stats.Manifolds, 10, "9 cube-cube + 1 ground-cube manifolds once the stack has formed");
        }

        [Test]
        public void RunsAreBitwiseDeterministic()
        {
            using var a = GpuTestUtil.NewWorld();
            using var b = GpuTestUtil.NewWorld();
            a.BuildScene(AvbdScenes.Pyramid);
            b.BuildScene(AvbdScenes.Pyramid);
            for (int i = 0; i < 90; i++) { a.Step(); b.Step(); }
            a.GetPosesSync(out var pa, out var ra);
            b.GetPosesSync(out var pb, out var rb);
            for (int i = 0; i < a.BodyCount; i++)
            {
                Assert.IsTrue(math.all(pa[i] == pb[i]), $"body {i} position differs: {pa[i]} vs {pb[i]}");
                Assert.IsTrue(math.all(ra[i] == rb[i]), $"body {i} rotation differs");
            }
        }
    }
}
