using System;
using UnityEngine;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>A siege: the castle (a brick-assembly document) and the two armies as rosters of unit configs with their counts, the side
    /// the attackers come from and how they form up. The siege scene lists every SiegeConfig asset of Resources on its number keys.
    /// Created through Phys / Siege / Siege Config or by Phys / Create Siege Configs.</summary>
    [CreateAssetMenu(menuName = "Phys/Siege/Siege Config", fileName = "SiegeConfig")]
    public sealed class SiegeConfig : ScriptableObject
    {
        [Serializable]
        public struct Roster
        {
            public UnitConfig Unit;
            public int Count;
        }

        public string DisplayName = "Siege";
        [Tooltip("The castle: a brick-assembly document (Resources/Castles/*.json).")]
        public TextAsset Castle;
        [Tooltip("The player's army, formed up on the attacked side.")]
        public Roster[] Attackers = new Roster[0];
        [Tooltip("The garrison, placed on the castle's posts (its walls first).")]
        public Roster[] Defenders = new Roster[0];
        [Range(0, 3)] [Tooltip("The side the attackers come from: 0 = -z (the gate side of the planned castles), 1 = +x, 2 = +z, 3 = -x.")]
        public int AttackSide;
        [Tooltip("Distance (solver metres) from the castle's face to the first rank.")]
        public float FormationDistance = 30f;
        [Tooltip("Spacing (solver metres) between ranks and between columns (at least 1.4 footprints).")]
        public float RankSpacing = 3f;
        public float ColumnSpacing = 2.5f;
        [Tooltip("Units per row of a rank.")]
        public int MaxColumns = 12;
        [Tooltip("The defenders' tint (alpha 0: each unit config's own tint).")]
        public Color32 DefenderTint = new Color32(70, 110, 200, 255);

        /// <summary>The outward direction of the attacked side.</summary>
        public Vector3 Outward => AttackSide == 1 ? Vector3.right : AttackSide == 2 ? Vector3.forward : AttackSide == 3 ? Vector3.left : Vector3.back;

        public int AttackerCount { get { int n = 0; foreach (var r in Attackers) if (r.Unit != null) n += Mathf.Max(r.Count, 0); return n; } }
        public int DefenderCount { get { int n = 0; foreach (var r in Defenders) if (r.Unit != null) n += Mathf.Max(r.Count, 0); return n; } }
    }
}
