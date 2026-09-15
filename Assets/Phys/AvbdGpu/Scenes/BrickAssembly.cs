// A brick-assembly document (Assets/Phys/BRICK_ASSEMBLY.md): JSON placements of the 27 pieces of the construction pack,
// expanded through their modules into a flat list of parts and then, like BrickCastle's stud-grid layout, into solver boxes.
// Every piece is a box body (sloped and round pieces take their bounding box); its model's studs nest in the hollow underside
// of the piece above, so the box is the body without the studs, grown by one collision margin on the side that faces down.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Scenes
{
    /// <summary>One of the 27 construction pieces (Assets/Models/construction_pieces): footprint and body height in grid units
    /// (1 = the stud pitch = 0.1 m), whether its top carries studs and whether its underside grips the studs it rests on.</summary>
    public struct Piece
    {
        public string Id;
        /// <summary>Extents of the body along local X, Z and Y (studs excluded), grid units.</summary>
        public float Width, Length, Height;
        /// <summary>Studs on top: a piece resting on it at the body-height pitch can be snapped to it.</summary>
        public bool Studs;
        /// <summary>Grips the studs it rests on (every piece but the plain shapes, which are solid decorative blocks).</summary>
        public bool Grips;

        public float3 Size => new float3(Width, Height, Length);
    }

    /// <summary>The catalog generic-construction-27-v1: IDs are the FBX file names, dimensions the nominal ones of the format.</summary>
    public static class PieceCatalog
    {
        public const string Id = "generic-construction-27-v1";
        /// <summary>Metres per grid unit: the stud pitch.</summary>
        public const float GridToUnity = 0.1f;
        /// <summary>Stud height in grid units (a snap comes apart once the pieces separate by half of it).</summary>
        public const float StudHeight = 0.225f;

        public static readonly Piece[] Pieces =
        {
            Brick(1, 1), Brick(1, 2), Brick(1, 3), Brick(1, 4), Brick(1, 6), Brick(1, 8),
            Brick(2, 2), Brick(2, 3), Brick(2, 4), Brick(2, 6), Brick(2, 8),
            Plate(1, 1), Plate(1, 2), Plate(1, 4), Plate(2, 2), Plate(2, 4),
            Tile(1, 1), Tile(1, 2), Tile(2, 2), Tile(2, 4),
            new Piece { Id = "Ramp", Width = 2, Length = 2, Height = 1.2f, Grips = true },
            new Piece { Id = "Roof_Prism", Width = 2, Length = 2, Height = 1.2f, Grips = true },
            new Piece { Id = "Wedge", Width = 2, Length = 3, Height = 0.8f, Grips = true },
            new Piece { Id = "Plain_Cube", Width = 1, Length = 1, Height = 1 },
            new Piece { Id = "Plain_Rectangular_Block", Width = 2, Length = 3, Height = 1.2f },
            new Piece { Id = "Plain_Cylinder", Width = 1.5f, Length = 1.5f, Height = 1.5f },
            new Piece { Id = "Plain_Triangular_Prism", Width = 2, Length = 2, Height = 1.2f },
        };

        static Piece Brick(int w, int l) => new Piece { Id = $"Brick_{w}x{l}", Width = w, Length = l, Height = 1.2f, Studs = true, Grips = true };
        static Piece Plate(int w, int l) => new Piece { Id = $"Plate_{w}x{l}", Width = w, Length = l, Height = 0.4f, Studs = true, Grips = true };
        static Piece Tile(int w, int l) => new Piece { Id = $"Tile_{w}x{l}", Width = w, Length = l, Height = 0.4f, Grips = true };

        /// <summary>Index of the piece in <see cref="Pieces"/>, or -1.</summary>
        public static int IndexOf(string id)
        {
            for (int i = 0; i < Pieces.Length; i++) if (Pieces[i].Id == id) return i;
            return -1;
        }
    }

    /// <summary>A placed piece of an expanded assembly: the pivot (bottom-face centre) in grid units, the composed rotation, the colour.</summary>
    public struct AssemblyPart
    {
        /// <summary>The expanded ID path, e.g. pillar_left/bottom.</summary>
        public string Id;
        /// <summary>Index into <see cref="PieceCatalog.Pieces"/>.</summary>
        public int Piece;
        public float3 Position;
        public quaternion Rotation;
        /// <summary>0xRRGGBB.</summary>
        public uint Rgb;
    }

    /// <summary>A document the loader rejects (malformed JSON, unsupported format, invalid vectors, duplicate IDs, unresolved
    /// references, circular modules ...); the message names the offending scope and ID.</summary>
    public sealed class BrickAssemblyException : Exception
    {
        public BrickAssemblyException(string message) : base(message) { }
    }

    /// <summary>A parsed and expanded brick-assembly document.</summary>
    public sealed class BrickAssembly
    {
        public const string Format = "brick-assembly";
        public const int Version = 1;
        public const uint DefaultRgb = 0xA0A0A0;

        public string Name = "";
        public readonly List<AssemblyPart> Parts = new List<AssemblyPart>();
        /// <summary>Bounds of the pieces' bodies in grid units (the boxes rotated into the world, studs excluded).</summary>
        public float3 Min, Max;

        public float3 Extent => Max - Min;

        /// <summary>Index of the part with the expanded ID, or -1.</summary>
        public int IndexOf(string id)
        {
            for (int i = 0; i < Parts.Count; i++) if (Parts[i].Id == id) return i;
            return -1;
        }

        /// <summary>Number of parts using each piece, most used first.</summary>
        public List<(int piece, int count)> PieceCounts()
        {
            var counts = new int[PieceCatalog.Pieces.Length];
            foreach (var p in Parts) counts[p.Piece]++;
            var result = new List<(int, int)>();
            for (int i = 0; i < counts.Length; i++) if (counts[i] > 0) result.Add((i, counts[i]));
            result.Sort((a, b) => b.Item2.CompareTo(a.Item2));
            return result;
        }

        // ------------------------------------------------------------------------------------------------ parsing

        sealed class PartDef { public string Id; public int Piece; public float3 Position; public quaternion Rotation; public uint Rgb; }
        sealed class InstanceDef { public string Id; public string Module; public float3 Position; public quaternion Rotation; }
        sealed class Scope { public readonly List<PartDef> Parts = new List<PartDef>(); public readonly List<InstanceDef> Instances = new List<InstanceDef>(); }

        /// <summary>Parses and expands a document; throws <see cref="BrickAssemblyException"/> for anything the format rejects.</summary>
        public static BrickAssembly Parse(string json)
        {
            if (!(JsonReader.Parse(json) is Dictionary<string, object> root)) throw new BrickAssemblyException("the document is not a JSON object");
            var a = new BrickAssembly();
            string format = GetString(root, "format", "the document");
            if (format != Format) throw new BrickAssemblyException($"unsupported format '{format}' (expected '{Format}')");
            double version = GetNumber(root, "version", "the document");
            if (version != Version) throw new BrickAssemblyException($"unsupported version {version} (expected {Version})");
            string catalog = GetString(root, "catalog", "the document");
            if (catalog != PieceCatalog.Id) throw new BrickAssemblyException($"unsupported catalog '{catalog}' (expected '{PieceCatalog.Id}')");
            a.Name = GetString(root, "name", "the document");
            var units = GetObject(root, "units", "the document", true);
            double gridToUnity = GetNumber(units, "gridToUnity", "units");
            if (math.abs(gridToUnity - PieceCatalog.GridToUnity) > 1e-6) throw new BrickAssemblyException($"units.gridToUnity {gridToUnity} is not {PieceCatalog.GridToUnity} (the only value for this catalog)");
            string axes = GetString(units, "axes", "units");
            if (axes != "unity") throw new BrickAssemblyException($"units.axes '{axes}' is not 'unity' (the only value for this catalog)");

            var palette = new Dictionary<string, uint>();
            var paletteObj = GetObject(root, "palette", "the document", false);
            if (paletteObj != null)
                foreach (var kv in paletteObj)
                    palette[kv.Key] = ParseHexColor(kv.Value as string, $"palette entry '{kv.Key}'");

            var modules = new Dictionary<string, Scope>();
            var modulesObj = GetObject(root, "modules", "the document", false);
            if (modulesObj != null)
                foreach (var kv in modulesObj)
                {
                    if (!(kv.Value is Dictionary<string, object> obj)) throw new BrickAssemblyException($"module '{kv.Key}' is not an object");
                    modules[kv.Key] = ParseScope(obj, $"module '{kv.Key}'", palette, false);
                }
            var rootScope = ParseScope(root, "the document", palette, true);

            // every module reference resolves and no module contains itself, whether instantiated or not
            var state = new Dictionary<string, int>();   // 1 = on the current path, 2 = done
            foreach (var kv in modules) CheckModuleGraph(kv.Key, modules, state);
            foreach (var inst in rootScope.Instances)
                if (!modules.ContainsKey(inst.Module)) throw new BrickAssemblyException($"instance '{inst.Id}' references unknown module '{inst.Module}'");

            a.Expand(rootScope, "", float3.zero, quaternion.identity, modules);
            var seen = new HashSet<string>();
            foreach (var p in a.Parts)
                if (!seen.Add(p.Id)) throw new BrickAssemblyException($"expanded part ID '{p.Id}' is not unique (IDs with slashes collide with instance paths)");
            a.ComputeBounds();
            return a;
        }

        static void CheckModuleGraph(string name, Dictionary<string, Scope> modules, Dictionary<string, int> state)
        {
            if (state.TryGetValue(name, out int s))
            {
                if (s == 1) throw new BrickAssemblyException($"circular module reference through '{name}'");
                return;
            }
            state[name] = 1;
            foreach (var inst in modules[name].Instances)
            {
                if (!modules.ContainsKey(inst.Module)) throw new BrickAssemblyException($"instance '{inst.Id}' of module '{name}' references unknown module '{inst.Module}'");
                CheckModuleGraph(inst.Module, modules, state);
            }
            state[name] = 2;
        }

        static Scope ParseScope(Dictionary<string, object> obj, string where, Dictionary<string, uint> palette, bool partsRequired)
        {
            var scope = new Scope();
            var ids = new HashSet<string>();
            var parts = GetArray(obj, "parts", where, partsRequired);
            if (parts != null)
                for (int i = 0; i < parts.Count; i++)
                {
                    if (!(parts[i] is Dictionary<string, object> p)) throw new BrickAssemblyException($"part {i} of {where} is not an object");
                    string id = GetString(p, "id", $"part {i} of {where}");
                    string here = $"part '{id}' of {where}";
                    if (!ids.Add(id)) throw new BrickAssemblyException($"duplicate ID '{id}' in {where}");
                    string piece = GetString(p, "piece", here);
                    int index = PieceCatalog.IndexOf(piece);
                    if (index < 0) throw new BrickAssemblyException($"{here}: unknown piece '{piece}'");
                    var def = new PartDef { Id = id, Piece = index, Position = GetVector(p, "position", here, true), Rotation = Rotation(GetVector(p, "rotation", here, false)) };
                    object color = p.TryGetValue("color", out object c) ? c : null;
                    if (color == null) def.Rgb = DefaultRgb;
                    else if (!(color is string s)) throw new BrickAssemblyException($"{here}: color is not a string");
                    else if (s.StartsWith("#", StringComparison.Ordinal)) def.Rgb = ParseHexColor(s, here);
                    else if (palette.TryGetValue(s, out uint rgb)) def.Rgb = rgb;
                    else throw new BrickAssemblyException($"{here}: color '{s}' is not in the palette");
                    scope.Parts.Add(def);
                }
            var instances = GetArray(obj, "instances", where, false);
            if (instances != null)
                for (int i = 0; i < instances.Count; i++)
                {
                    if (!(instances[i] is Dictionary<string, object> inst)) throw new BrickAssemblyException($"instance {i} of {where} is not an object");
                    string id = GetString(inst, "id", $"instance {i} of {where}");
                    string here = $"instance '{id}' of {where}";
                    if (!ids.Add(id)) throw new BrickAssemblyException($"duplicate ID '{id}' in {where}");
                    scope.Instances.Add(new InstanceDef { Id = id, Module = GetString(inst, "module", here), Position = GetVector(inst, "position", here, true), Rotation = Rotation(GetVector(inst, "rotation", here, false)) });
                }
            return scope;
        }

        void Expand(Scope scope, string prefix, float3 position, quaternion rotation, Dictionary<string, Scope> modules)
        {
            foreach (var p in scope.Parts)
                Parts.Add(new AssemblyPart { Id = prefix + p.Id, Piece = p.Piece, Position = position + math.mul(rotation, p.Position), Rotation = math.mul(rotation, p.Rotation), Rgb = p.Rgb });
            foreach (var inst in scope.Instances)
                Expand(modules[inst.Module], prefix + inst.Id + "/", position + math.mul(rotation, inst.Position), math.mul(rotation, inst.Rotation), modules);
        }

        void ComputeBounds()
        {
            Min = float.MaxValue; Max = float.MinValue;
            if (Parts.Count == 0) { Min = Max = float3.zero; return; }
            foreach (var p in Parts)
            {
                float3 half = PieceCatalog.Pieces[p.Piece].Size * 0.5f;
                float3 centre = p.Position + math.mul(p.Rotation, new float3(0f, half.y, 0f));
                float3 extent = math.abs(math.mul(p.Rotation, new float3(half.x, 0f, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, half.y, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, 0f, half.z)));
                Min = math.min(Min, centre - extent);
                Max = math.max(Max, centre + extent);
            }
        }

        /// <summary>The format's rotation: [x, y, z] degrees applied Z first, then X, then Y (Unity's Euler convention).</summary>
        public static quaternion Rotation(float3 degrees) => math.all(degrees == 0f) ? quaternion.identity : quaternion.EulerZXY(math.radians(degrees));

        // ------------------------------------------------------------------------------------------------ writing

        /// <summary>The castle planner's tones as palette colours (the castle demo's base shades).</summary>
        static readonly string[] s_ToneNames = { "stone", "slate", "tan", "red", "wood" };
        static readonly string[] s_ToneColors = { "#9EA0A4", "#484E5A", "#CDB280", "#A82C24", "#86603A" };

        /// <summary>Writes a stud-grid layout of 2 x 3 bricks (<see cref="BrickCastle.Generate"/>) as a brick-assembly document: one
        /// part per brick, courses at the body-height pitch, rotated bricks turned +90 degrees about Y, tones as the palette.</summary>
        public static string WriteLayout(BrickLayout layout, string name)
        {
            var sb = new StringBuilder(layout.Bricks.Count * 120 + 512);
            sb.Append("{\n  \"format\": \"").Append(Format).Append("\",\n  \"version\": ").Append(Version).Append(",\n  \"catalog\": \"").Append(PieceCatalog.Id).Append("\",\n");
            sb.Append("  \"name\": \"").Append(Escape(name)).Append("\",\n  \"units\": { \"gridToUnity\": 0.1, \"axes\": \"unity\" },\n  \"palette\": { ");
            for (int i = 0; i < s_ToneNames.Length; i++) sb.Append(i > 0 ? ", " : "").Append('"').Append(s_ToneNames[i]).Append("\": \"").Append(s_ToneColors[i]).Append('"');
            sb.Append(" },\n  \"parts\": [\n");
            for (int i = 0; i < layout.Bricks.Count; i++)
            {
                var b = layout.Bricks[i];
                float x = b.X + b.W * 0.5f, z = b.Z + b.D * 0.5f, y = b.Layer * 1.2f;
                sb.Append("    { \"id\": \"b").Append(i).Append("\", \"piece\": \"Brick_2x3\", \"position\": [")
                  .Append(Number(x)).Append(", ").Append(Number(y)).Append(", ").Append(Number(z)).Append(']');
                if (b.Rotated) sb.Append(", \"rotation\": [0, 90, 0]");
                sb.Append(", \"color\": \"").Append(s_ToneNames[math.clamp((int)b.Tone, 0, s_ToneNames.Length - 1)]).Append("\" }").Append(i + 1 < layout.Bricks.Count ? ",\n" : "\n");
            }
            sb.Append("  ]\n}\n");
            return sb.ToString();
        }

        static string Number(float v) => v.ToString(math.abs(v - math.round(v)) < 1e-6f ? "0" : "0.###", CultureInfo.InvariantCulture);
        static string Escape(string s) => s.Replace("\\", "\\\\").Replace("\"", "\\\"");

        static string GetString(Dictionary<string, object> obj, string key, string where)
        {
            if (!obj.TryGetValue(key, out object v) || v == null) throw new BrickAssemblyException($"{where}: missing '{key}'");
            if (!(v is string s)) throw new BrickAssemblyException($"{where}: '{key}' is not a string");
            return s;
        }

        static double GetNumber(Dictionary<string, object> obj, string key, string where)
        {
            if (!obj.TryGetValue(key, out object v) || v == null) throw new BrickAssemblyException($"{where}: missing '{key}'");
            if (!(v is double d)) throw new BrickAssemblyException($"{where}: '{key}' is not a number");
            return d;
        }

        static Dictionary<string, object> GetObject(Dictionary<string, object> obj, string key, string where, bool required)
        {
            if (!obj.TryGetValue(key, out object v) || v == null)
            {
                if (required) throw new BrickAssemblyException($"{where}: missing '{key}'");
                return null;
            }
            if (!(v is Dictionary<string, object> o)) throw new BrickAssemblyException($"{where}: '{key}' is not an object");
            return o;
        }

        static List<object> GetArray(Dictionary<string, object> obj, string key, string where, bool required)
        {
            if (!obj.TryGetValue(key, out object v) || v == null)
            {
                if (required) throw new BrickAssemblyException($"{where}: missing '{key}'");
                return null;
            }
            if (!(v is List<object> list)) throw new BrickAssemblyException($"{where}: '{key}' is not an array");
            return list;
        }

        static float3 GetVector(Dictionary<string, object> obj, string key, string where, bool required)
        {
            if (!obj.TryGetValue(key, out object v) || v == null)
            {
                if (required) throw new BrickAssemblyException($"{where}: missing '{key}'");
                return float3.zero;
            }
            if (!(v is List<object> list) || list.Count != 3) throw new BrickAssemblyException($"{where}: '{key}' is not three numbers");
            float3 result = float3.zero;
            for (int i = 0; i < 3; i++)
            {
                if (!(list[i] is double d) || double.IsNaN(d) || double.IsInfinity(d)) throw new BrickAssemblyException($"{where}: '{key}' is not three finite numbers");
                result[i] = (float)d;
            }
            return result;
        }

        static uint ParseHexColor(string s, string where)
        {
            if (s == null || s.Length != 7 || s[0] != '#' || !uint.TryParse(s.Substring(1), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out uint rgb))
                throw new BrickAssemblyException($"{where}: colour '{s}' is not #RRGGBB");
            return rgb;
        }
    }

    /// <summary>Minimal JSON reader for the documents: objects (Dictionary&lt;string, object&gt;), arrays (List&lt;object&gt;), strings,
    /// numbers (double), true, false and null. Duplicate keys are rejected.</summary>
    internal sealed class JsonReader
    {
        readonly string m_Text;
        int m_Pos;

        JsonReader(string text) { m_Text = text ?? ""; }

        public static object Parse(string text)
        {
            var r = new JsonReader(text);
            r.SkipWhitespace();
            object value = r.ReadValue();
            r.SkipWhitespace();
            if (r.m_Pos != r.m_Text.Length) throw r.Error("text after the document");
            return value;
        }

        BrickAssemblyException Error(string what)
        {
            int line = 1, column = 1;
            for (int i = 0; i < m_Pos && i < m_Text.Length; i++)
                if (m_Text[i] == '\n') { line++; column = 1; } else column++;
            return new BrickAssemblyException($"malformed JSON at line {line}, column {column}: {what}");
        }

        char Peek() => m_Pos < m_Text.Length ? m_Text[m_Pos] : '\0';

        void SkipWhitespace()
        {
            while (m_Pos < m_Text.Length && (m_Text[m_Pos] == ' ' || m_Text[m_Pos] == '\t' || m_Text[m_Pos] == '\n' || m_Text[m_Pos] == '\r')) m_Pos++;
        }

        object ReadValue()
        {
            char c = Peek();
            switch (c)
            {
                case '{': return ReadObject();
                case '[': return ReadArray();
                case '"': return ReadString();
                case 't': Expect("true"); return true;
                case 'f': Expect("false"); return false;
                case 'n': Expect("null"); return null;
                case '\0': throw Error("unexpected end of text");
                default:
                    if (c == '-' || (c >= '0' && c <= '9')) return ReadNumber();
                    throw Error($"unexpected '{c}'");
            }
        }

        void Expect(string word)
        {
            if (string.CompareOrdinal(m_Text, m_Pos, word, 0, word.Length) != 0) throw Error($"expected '{word}'");
            m_Pos += word.Length;
        }

        Dictionary<string, object> ReadObject()
        {
            var obj = new Dictionary<string, object>();
            m_Pos++;
            SkipWhitespace();
            if (Peek() == '}') { m_Pos++; return obj; }
            while (true)
            {
                SkipWhitespace();
                if (Peek() != '"') throw Error("expected a string key");
                string key = ReadString();
                SkipWhitespace();
                if (Peek() != ':') throw Error("expected ':'");
                m_Pos++;
                SkipWhitespace();
                if (obj.ContainsKey(key)) throw Error($"duplicate key '{key}'");
                obj[key] = ReadValue();
                SkipWhitespace();
                char c = Peek();
                m_Pos++;
                if (c == ',') continue;
                if (c == '}') return obj;
                throw Error("expected ',' or '}'");
            }
        }

        List<object> ReadArray()
        {
            var list = new List<object>();
            m_Pos++;
            SkipWhitespace();
            if (Peek() == ']') { m_Pos++; return list; }
            while (true)
            {
                SkipWhitespace();
                list.Add(ReadValue());
                SkipWhitespace();
                char c = Peek();
                m_Pos++;
                if (c == ',') continue;
                if (c == ']') return list;
                throw Error("expected ',' or ']'");
            }
        }

        string ReadString()
        {
            m_Pos++;   // the opening quote
            var sb = new StringBuilder();
            while (true)
            {
                if (m_Pos >= m_Text.Length) throw Error("unterminated string");
                char c = m_Text[m_Pos++];
                if (c == '"') return sb.ToString();
                if (c < ' ') throw Error("control character in a string");
                if (c != '\\') { sb.Append(c); continue; }
                if (m_Pos >= m_Text.Length) throw Error("unterminated string");
                char e = m_Text[m_Pos++];
                switch (e)
                {
                    case '"': sb.Append('"'); break;
                    case '\\': sb.Append('\\'); break;
                    case '/': sb.Append('/'); break;
                    case 'b': sb.Append('\b'); break;
                    case 'f': sb.Append('\f'); break;
                    case 'n': sb.Append('\n'); break;
                    case 'r': sb.Append('\r'); break;
                    case 't': sb.Append('\t'); break;
                    case 'u':
                        if (m_Pos + 4 > m_Text.Length || !int.TryParse(m_Text.Substring(m_Pos, 4), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out int code)) throw Error("bad \\u escape");
                        sb.Append((char)code);
                        m_Pos += 4;
                        break;
                    default: throw Error($"bad escape '\\{e}'");
                }
            }
        }

        double ReadNumber()
        {
            int start = m_Pos;
            if (Peek() == '-') m_Pos++;
            int digits = 0;
            while (Peek() >= '0' && Peek() <= '9') { m_Pos++; digits++; }
            if (digits == 0) throw Error("expected a number");
            if (Peek() == '.')
            {
                m_Pos++;
                int fraction = 0;
                while (Peek() >= '0' && Peek() <= '9') { m_Pos++; fraction++; }
                if (fraction == 0) throw Error("expected digits after '.'");
            }
            if (Peek() == 'e' || Peek() == 'E')
            {
                m_Pos++;
                if (Peek() == '+' || Peek() == '-') m_Pos++;
                int exponent = 0;
                while (Peek() >= '0' && Peek() <= '9') { m_Pos++; exponent++; }
                if (exponent == 0) throw Error("expected digits in the exponent");
            }
            if (!double.TryParse(m_Text.Substring(start, m_Pos - start), NumberStyles.Float, CultureInfo.InvariantCulture, out double value) || double.IsInfinity(value))
                throw Error("number out of range");
            return value;
        }
    }

    /// <summary>How an assembly maps to solver bodies (the counterpart of <see cref="BrickSpec"/>): model scale, material, and where
    /// the grid origin stands.</summary>
    public struct AssemblySpec
    {
        /// <summary>Solver metres per model metre (1 = the true 0.1 m stud pitch).</summary>
        public float Scale;
        public float Density, Friction;
        /// <summary>The solver's collision margin: resting contacts settle this deep, so every box is that much taller on the side that
        /// faces down and the pieces stack exactly at their body heights.</summary>
        public float Margin;
        /// <summary>Clearance of the collision box from the piece's footprint, per side, in model metres (the two axes across the one that
        /// faces down). The pack's pieces fill their stud pitch exactly; real bricks are about 1.25 % narrower, and side-by-side boxes
        /// that touch pass loads that snapped bricks are not built to take. The models are drawn at full size.</summary>
        public float Clearance;
        /// <summary>World position of grid point (0, 0, 0).</summary>
        public float3 Origin;

        /// <summary>1.25 % of the stud pitch: 1.25 mm per side of the model, 1.25 cm between neighbours at scale 5 (more than the margin).</summary>
        public const float DefaultClearance = 0.00125f;

        public static AssemblySpec Default => new AssemblySpec { Scale = 1f, Density = 1050f, Friction = 0.6f, Margin = BrickSpec.CollisionMargin, Clearance = DefaultClearance, Origin = float3.zero };

        /// <summary>Solver metres per grid unit.</summary>
        public float Unit => PieceCatalog.GridToUnity * Scale;
        /// <summary>Mass of a 2 x 3 brick at this density.</summary>
        public float BrickMass => 2f * 3f * 1.2f * Unit * Unit * Unit * Density;
        /// <summary>A snap comes apart once the pieces separate by half the stud height.</summary>
        public float SnapBreakDistance => 0.5f * PieceCatalog.StudHeight * Unit;
    }

    /// <summary>The bodies of a built assembly: one box per part, laid out in groups that share a piece and a mesh offset (one
    /// draw range each), with the maps between parts and bodies.</summary>
    public sealed class AssemblyBodies
    {
        public struct MeshGroup
        {
            public int Piece;
            /// <summary>The model pivot relative to the box centre, in body space.</summary>
            public float3 MeshOffset;
            public int Start, Count;
        }

        public int First, Count;
        public int[] BodyOfPart;
        /// <summary>Part index of body First + i.</summary>
        public int[] PartOfBody;
        public readonly List<MeshGroup> Groups = new List<MeshGroup>();
    }

    /// <summary>Turns an expanded assembly into solver boxes and snap joints, the way <see cref="BrickCastle"/> does for its layout.</summary>
    public static class AssemblyBuilder
    {
        /// <summary>Pieces meeting within this distance (grid units) of the body-height pitch are snapped.</summary>
        public const float SnapTolerance = 0.02f;
        /// <summary>Least footprint overlap (studs, in both directions) for a snap: a stud must sit in the cavity above.</summary>
        public const float SnapMinOverlap = 0.5f;

        /// <summary>The collision box of a part: the piece body (studs excluded) grown by one margin along the body axis that faces
        /// down, so that a resting contact, which settles one margin deep, leaves the model exactly on the surface below, and shrunk
        /// by the clearance on the two other axes. The position is the box centre; the mesh offset is the model pivot relative to
        /// it, in body space.</summary>
        public static void Box(in AssemblyPart p, in AssemblySpec spec, out float3 size, out float3 position, out float3 meshOffset)
        {
            float3 body = PieceCatalog.Pieces[p.Piece].Size * spec.Unit;
            float3 d = DownAxis(p.Rotation);
            float3 across = 1f - math.abs(d);
            size = body + math.abs(d) * spec.Margin - across * (2f * spec.Clearance * spec.Scale);
            float3 centre = new float3(0f, body.y * 0.5f, 0f) + d * (spec.Margin * 0.5f);
            position = spec.Origin + p.Position * spec.Unit + math.mul(p.Rotation, centre);
            meshOffset = -centre;
        }

        /// <summary>The body axis (one of +-x, +-y, +-z) closest to the world's down direction; -y for an upright piece.</summary>
        public static float3 DownAxis(quaternion rotation)
        {
            float3 down = math.mul(math.conjugate(rotation), new float3(0f, -1f, 0f));
            float3 a = math.abs(down);
            int axis = a.y >= a.x && a.y >= a.z ? 1 : a.x >= a.z ? 0 : 2;
            float3 d = float3.zero;
            d[axis] = down[axis] < 0f ? -1f : 1f;
            return d;
        }

        static int GroupKey(in AssemblyPart p)
        {
            float3 d = DownAxis(p.Rotation);
            int axis = d.x != 0f ? 0 : d.y != 0f ? 1 : 2;
            return p.Piece * 6 + axis * 2 + (d[axis] < 0f ? 0 : 1);
        }

        /// <summary>Adds every part as a box, ordered so that parts sharing a piece and a mesh offset are consecutive.</summary>
        public static AssemblyBodies Build(ISceneBuilder s, BrickAssembly a, AssemblySpec spec)
        {
            int n = a.Parts.Count;
            var keys = new int[n];
            var order = new int[n];
            for (int i = 0; i < n; i++) { keys[i] = GroupKey(a.Parts[i]); order[i] = i; }
            Array.Sort(order, (x, y) => keys[x] != keys[y] ? keys[x].CompareTo(keys[y]) : x.CompareTo(y));
            var result = new AssemblyBodies { Count = n, BodyOfPart = new int[n], PartOfBody = new int[n] };
            int previous = int.MinValue;
            for (int k = 0; k < n; k++)
            {
                int i = order[k];
                var p = a.Parts[i];
                Box(p, spec, out float3 size, out float3 position, out float3 offset);
                int body = s.AddBody(size, spec.Density, spec.Friction, position, p.Rotation, float3.zero);
                if (k == 0) result.First = body;
                result.BodyOfPart[i] = body;
                result.PartOfBody[body - result.First] = i;
                if (keys[i] != previous)
                {
                    result.Groups.Add(new AssemblyBodies.MeshGroup { Piece = p.Piece, MeshOffset = offset, Start = body });
                    previous = keys[i];
                }
                var g = result.Groups[result.Groups.Count - 1];
                g.Count++;
                result.Groups[result.Groups.Count - 1] = g;
            }
            return result;
        }

        /// <summary>Snaps the pieces together like <see cref="BrickCastle.AddSnapJoints"/>: four hard ball-socket joints at the inset
        /// corners of every overlap in which an upright, stud-aligned piece rests exactly one body height on a studded top (bricks and
        /// plates), and between every gripping piece on the ground and the world. The limits are split over the four joints.</summary>
        public static int AddSnapJoints(ISceneBuilder s, BrickAssembly a, AssemblyBodies bodies, AssemblySpec spec, float fractureLateral, float fractureTension)
        {
            const int Anchors = 4;
            float lateral = fractureLateral / Anchors, tension = fractureTension / Anchors, distance = spec.SnapBreakDistance;
            int n = a.Parts.Count;
            var rect = new float4[n];       // footprint [xMin, zMin, xMax, zMax] in grid units of the upright, stud-aligned parts
            var upright = new bool[n];
            var top = new float[n];
            var byTop = new Dictionary<int, List<int>>();   // studded parts by the height of their top plane
            for (int i = 0; i < n; i++)
            {
                var p = a.Parts[i];
                var piece = PieceCatalog.Pieces[p.Piece];
                float3 up = math.mul(p.Rotation, new float3(0f, 1f, 0f)), x = math.mul(p.Rotation, new float3(1f, 0f, 0f));
                bool alongX = math.abs(x.x) > 0.999f;
                upright[i] = up.y > 0.999f && (alongX || math.abs(x.z) > 0.999f);
                if (!upright[i]) continue;
                float2 half = alongX ? new float2(piece.Width, piece.Length) * 0.5f : new float2(piece.Length, piece.Width) * 0.5f;
                rect[i] = new float4(p.Position.xz - half, p.Position.xz + half);
                top[i] = p.Position.y + piece.Height;
                if (!piece.Studs) continue;
                int key = Key(top[i]);
                if (!byTop.TryGetValue(key, out var list)) byTop[key] = list = new List<int>();
                list.Add(i);
            }
            int joints = 0;
            for (int i = 0; i < n; i++)
            {
                if (!upright[i] || !PieceCatalog.Pieces[a.Parts[i].Piece].Grips) continue;
                var b = a.Parts[i];
                int bodyB = bodies.BodyOfPart[i];
                Box(b, spec, out _, out float3 posB, out _);
                quaternion invB = math.conjugate(b.Rotation);
                float bottom = b.Position.y;
                if (math.abs(bottom) < SnapTolerance)   // on the ground: snapped to the world, like the castle's ground course
                {
                    foreach (var c in InsetCorners(rect[i]))
                    {
                        float3 anchor = spec.Origin + new float3(c.x, 0f, c.y) * spec.Unit;
                        s.AddJoint(-1, bodyB, anchor, math.mul(invB, anchor - posB), float.PositiveInfinity, 0f, float.PositiveInfinity, lateral, tension, distance, BrickCastle.SnapAxisUp);
                        joints++;
                    }
                    continue;
                }
                int key = Key(bottom);
                for (int dk = -1; dk <= 1; dk++)
                {
                    if (!byTop.TryGetValue(key + dk, out var candidates)) continue;
                    foreach (int j in candidates)
                    {
                        if (j == i || math.abs(top[j] - bottom) > SnapTolerance) continue;
                        var overlap = new float4(math.max(rect[i].xy, rect[j].xy), math.min(rect[i].zw, rect[j].zw));
                        if (overlap.z - overlap.x < SnapMinOverlap || overlap.w - overlap.y < SnapMinOverlap) continue;
                        var below = a.Parts[j];
                        int bodyA = bodies.BodyOfPart[j];
                        Box(below, spec, out _, out float3 posA, out _);
                        quaternion invA = math.conjugate(below.Rotation);
                        foreach (var c in InsetCorners(overlap))
                        {
                            float3 anchor = spec.Origin + new float3(c.x, top[j], c.y) * spec.Unit;
                            s.AddJoint(bodyA, bodyB, math.mul(invA, anchor - posA), math.mul(invB, anchor - posB), float.PositiveInfinity, 0f, float.PositiveInfinity, lateral, tension, distance, BrickCastle.SnapAxisUp);
                            joints++;
                        }
                    }
                }
            }
            return joints;
        }

        /// <summary>Height bucket of a plane (buckets of the snap tolerance; the neighbouring buckets are searched as well).</summary>
        static int Key(float y) => (int)math.round(y / SnapTolerance);

        /// <summary>What the format asks a loader to report without touching the placements: pieces resting on nothing, pieces resting
        /// on less than half their footprint, and pairs of intersecting bodies. A piece counts as resting on another when its bottom
        /// lies on the other's top plane or a stud height above it; the ground supports everything at y = 0.</summary>
        public struct Diagnostics
        {
            public int Floating, PoorlySupported, Intersections;
            /// <summary>The first two offending IDs (one HUD line's worth).</summary>
            public string Sample;
            public bool Clean => Floating == 0 && PoorlySupported == 0 && Intersections == 0;
        }

        public static Diagnostics Diagnose(BrickAssembly a)
        {
            const float Tolerance = 0.05f, Cell = 8f;
            int n = a.Parts.Count;
            var min = new float3[n]; var max = new float3[n];
            var cells = new Dictionary<long, List<int>>();
            for (int i = 0; i < n; i++)
            {
                var p = a.Parts[i];
                float3 half = PieceCatalog.Pieces[p.Piece].Size * 0.5f;
                float3 centre = p.Position + math.mul(p.Rotation, new float3(0f, half.y, 0f));
                float3 extent = math.abs(math.mul(p.Rotation, new float3(half.x, 0f, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, half.y, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, 0f, half.z)));
                min[i] = centre - extent; max[i] = centre + extent;
                for (int cx = (int)math.floor(min[i].x / Cell); cx <= (int)math.floor(max[i].x / Cell); cx++)
                    for (int cz = (int)math.floor(min[i].z / Cell); cz <= (int)math.floor(max[i].z / Cell); cz++)
                    {
                        long key = ((long)cx << 32) ^ (uint)cz;
                        if (!cells.TryGetValue(key, out var list)) cells[key] = list = new List<int>();
                        list.Add(i);
                    }
            }
            var d = new Diagnostics();
            var sample = new List<string>();
            var seen = new HashSet<long>();
            for (int i = 0; i < n; i++)
            {
                float support = 0f, footprint = (max[i].x - min[i].x) * (max[i].z - min[i].z);
                bool onGround = min[i].y < Tolerance;
                for (int cx = (int)math.floor(min[i].x / Cell); cx <= (int)math.floor(max[i].x / Cell); cx++)
                    for (int cz = (int)math.floor(min[i].z / Cell); cz <= (int)math.floor(max[i].z / Cell); cz++)
                        foreach (int j in cells[((long)cx << 32) ^ (uint)cz])
                        {
                            if (j == i) continue;
                            float ox = math.min(max[i].x, max[j].x) - math.max(min[i].x, min[j].x), oz = math.min(max[i].z, max[j].z) - math.max(min[i].z, min[j].z);
                            if (ox <= 0f || oz <= 0f) continue;
                            float gap = min[i].y - max[j].y;   // the piece's bottom above the other's top
                            if (gap > -Tolerance && gap < PieceCatalog.StudHeight + Tolerance && !seenPair(i, j)) support += ox * oz;
                            if (j > i && math.min(max[i].y, max[j].y) - math.max(min[i].y, min[j].y) > 0.01f && ox > 0.01f && oz > 0.01f && seen.Add(((long)i << 32) | (uint)j))
                            {
                                d.Intersections++;
                                if (sample.Count < 2) sample.Add($"{a.Parts[i].Id} x {a.Parts[j].Id}");
                            }
                        }
                if (onGround) continue;
                if (support < 1e-6f) { d.Floating++; if (sample.Count < 2) sample.Add(a.Parts[i].Id); }
                else if (support < 0.5f * footprint - 1e-3f) d.PoorlySupported++;   // exactly half (a lintel on two piers) is not less than half
            }
            d.Sample = string.Join(", ", sample);
            return d;

            // a supporter overlapping several of the piece's cells is met once per cell: count its area once
            bool seenPair(int i, int j) => !seen.Add(((long)i << 32) | 0x8000_0000L | (uint)j);
        }

        /// <summary>The four corners of the rectangle [xMin, zMin, xMax, zMax] moved a quarter of the way to its centre.</summary>
        static float2[] InsetCorners(float4 r)
        {
            float dx = (r.z - r.x) * 0.25f, dz = (r.w - r.y) * 0.25f;
            return new[] { new float2(r.x + dx, r.y + dz), new float2(r.z - dx, r.y + dz), new float2(r.x + dx, r.w - dz), new float2(r.z - dx, r.w - dz) };
        }
    }
}
