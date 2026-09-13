using Unity.Mathematics;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Phys.Demo
{
    /// <summary>Orbit camera: right mouse drag orbits, middle drag pans, wheel zooms; A/D yaw, W/S pitch, Q/E zoom.</summary>
    public class DemoCamera : MonoBehaviour
    {
        public float3 Target = new float3(0f, 5f, 0f);
        public float Distance = 30f;
        public float Yaw = 35f;
        public float Pitch = 22f;
        public float OrbitSpeed = 0.25f;
        public float PanSpeed = 0.002f;
        public float ZoomSpeed = 0.1f;
        public float KeyOrbitSpeed = 120f;

        public void Frame(float3 target, float distance)
        {
            Target = target;
            Distance = distance;
            Apply();
        }

        void LateUpdate()
        {
#if ENABLE_INPUT_SYSTEM
            var mouse = Mouse.current;
            if (mouse != null)
            {
                float2 delta = mouse.delta.ReadValue();
                if (mouse.rightButton.isPressed)
                {
                    Yaw += delta.x * OrbitSpeed;
                    Pitch = math.clamp(Pitch - delta.y * OrbitSpeed, -85f, 85f);
                }
                if (mouse.middleButton.isPressed)
                {
                    float3 right = transform.right, up = transform.up;
                    Target -= (right * delta.x + up * delta.y) * (PanSpeed * Distance);
                }
                float scroll = mouse.scroll.ReadValue().y;
                if (math.abs(scroll) > 0f) Distance = math.clamp(Distance * (1f - math.sign(scroll) * ZoomSpeed), 0.5f, 2000f);
            }
            var kb = Keyboard.current;
            if (kb != null)
            {
                float d = KeyOrbitSpeed * Time.deltaTime;
                if (kb.aKey.isPressed) Yaw -= d;
                if (kb.dKey.isPressed) Yaw += d;
                if (kb.wKey.isPressed) Pitch = math.clamp(Pitch + d, -85f, 85f);
                if (kb.sKey.isPressed) Pitch = math.clamp(Pitch - d, -85f, 85f);
                if (kb.eKey.isPressed) Distance = math.min(Distance * 1.025f, 2000f);
                if (kb.qKey.isPressed) Distance = math.max(Distance / 1.025f, 0.5f);
            }
#endif
            Apply();
        }

        void Apply()
        {
            quaternion rot = quaternion.Euler(math.radians(Pitch), math.radians(Yaw), 0f);
            float3 offset = math.mul(rot, new float3(0f, 0f, -Distance));
            transform.SetPositionAndRotation(Target + offset, rot);
        }
    }
}
