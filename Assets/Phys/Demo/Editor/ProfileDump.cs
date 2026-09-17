using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Profiling;
using UnityEditorInternal;
using UnityEngine;

namespace Phys.Demo.Editor
{
    /// <summary>Dumps a Profiler capture (a .raw file written by a development player with -avbd-profile) as text: for every thread,
    /// the samples averaged per frame (total and self time, calls) with their paths, so that a bench can be read without the Profiler
    /// window. Batch: Unity.exe -batchmode -quit -executeMethod Phys.Demo.Editor.ProfileDump.Dump -profile C:/logs/x.raw [-profileOut
    /// C:/logs/x.txt] [-profileDepth 4] [-profileMin 0.02] [-profileThreads Main,Render].</summary>
    public static class ProfileDump
    {
        sealed class Acc { public double Total, Self; public long Calls; public int Frames; }

        public static void Dump()
        {
            var args = System.Environment.GetCommandLineArgs();
            string path = null, outPath = null;
            int depth = 4; double minMs = 0.02;
            string threadFilter = null;
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-profile") path = args[i + 1];
                if (args[i] == "-profileOut") outPath = args[i + 1];
                if (args[i] == "-profileDepth") int.TryParse(args[i + 1], out depth);
                if (args[i] == "-profileMin") double.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out minMs);
                if (args[i] == "-profileThreads") threadFilter = args[i + 1];
            }
            if (path == null) { Debug.LogError("ProfileDump: -profile <file.raw> missing"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            outPath ??= Path.ChangeExtension(path, ".txt");
            Dump(path, outPath, depth, minMs, threadFilter);
        }

        /// <summary><paramref name="threadFilter"/>: a comma-separated list of thread names (substrings) to dump, null for every thread.</summary>
        public static void Dump(string path, string outPath, int depth, double minMs, string threadFilter = null)
        {
            var wanted = threadFilter?.Split(',');
            if (!ProfilerDriver.LoadProfile(path, false)) { Debug.LogError($"ProfileDump: cannot load {path}"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            int first = ProfilerDriver.firstFrameIndex, last = ProfilerDriver.lastFrameIndex;
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"profile {path}: frames {first}..{last} ({last - first + 1})");
            // threads by index (the set is the same in every frame of one capture)
            var threads = new List<(int index, string name)>();
            for (int t = 0; t < 64; t++)
            {
                using var raw = ProfilerDriver.GetRawFrameDataView(first + (last - first) / 2, t);
                if (raw == null || !raw.valid) { if (t > 8) break; else continue; }
                threads.Add((t, raw.threadName));
            }
            var children = new List<int>();
            foreach (var (threadIndex, threadName) in threads)
            {
                if (wanted != null && !wanted.Any(w => threadName.Contains(w))) continue;
                var acc = new Dictionary<string, Acc>();
                int frames = 0;
                double frameMsSum = 0;
                for (int f = first; f <= last; f++)
                {
                    using var view = ProfilerDriver.GetHierarchyFrameDataView(f, threadIndex, HierarchyFrameDataView.ViewModes.MergeSamplesWithTheSameName, HierarchyFrameDataView.columnDontSort, false);
                    if (view == null || !view.valid) continue;
                    frames++;
                    frameMsSum += view.frameTimeMs;
                    Walk(view, view.GetRootItemID(), "", 0, depth, acc, children);
                }
                if (frames == 0) continue;
                sb.AppendLine();
                sb.AppendLine($"== thread {threadIndex} '{threadName}': {frames} frames, frame {frameMsSum / frames:F2} ms; per frame ms (total / self / calls), path (depth <= {depth}), samples >= {minMs} ms ==");
                foreach (var kv in acc.OrderByDescending(k => k.Value.Total))
                {
                    var a = kv.Value;
                    double total = a.Total / frames, self = a.Self / frames;
                    if (total < minMs) continue;
                    sb.AppendLine($"{total,9:F3} {self,9:F3} {(double)a.Calls / frames,8:F1}  {kv.Key}");
                }
            }
            File.WriteAllText(outPath, sb.ToString());
            Debug.Log($"ProfileDump: {threads.Count} threads -> {outPath}");
        }

        static void Walk(HierarchyFrameDataView view, int id, string path, int level, int maxDepth, Dictionary<string, Acc> acc, List<int> scratch)
        {
            var kids = new List<int>();
            view.GetItemChildren(id, kids);
            foreach (int c in kids)
            {
                string name = view.GetItemName(c);
                string p = level == 0 ? name : path + " / " + name;
                if (!acc.TryGetValue(p, out var a)) acc[p] = a = new Acc();
                a.Total += view.GetItemColumnDataAsFloat(c, HierarchyFrameDataView.columnTotalTime);
                a.Self += view.GetItemColumnDataAsFloat(c, HierarchyFrameDataView.columnSelfTime);
                a.Calls += (long)view.GetItemColumnDataAsFloat(c, HierarchyFrameDataView.columnCalls);
                if (level + 1 < maxDepth) Walk(view, c, p, level + 1, maxDepth, acc, scratch);
            }
        }
    }
}
