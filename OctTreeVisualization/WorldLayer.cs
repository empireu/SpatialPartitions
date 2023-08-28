using System.Numerics;
using Common;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities.Extensions;
using ImGuiNET;
using Veldrid;

namespace OctTreeVisualization;

internal sealed class WorldLayer : VisualizationLayer3d
{
    private readonly QuadBatch _treeBatch;
    private HashedBitOctree? _octree;

    private int _recreateLog = 2;
    private int _placeX;
    private int _placeY;
    private int _placeZ;
    
    public WorldLayer(VisualizationApp app, ImGuiLayer imGui) : base(app, imGui)
    {
        _treeBatch = GetBatch();

        CameraController.MoveSpeed = 5f;
    }

    protected override void UpdateBatchPipelines(OutputDescription outputDesc)
    {
        _treeBatch.UpdatePipelines(
            outputDescription: outputDesc, 
            blendState: BlendStateDescription.SingleAdditiveBlend, 
            depthStencilState: DepthStencilStateDescription.Disabled
        );
    }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Visualization"))
        {
            ImGui.SliderInt("Log", ref _recreateLog, 2, 10);

            if (ImGui.Button("Create"))
            {
                _octree = new HashedBitOctree((byte)_recreateLog);
            }

            if (_octree != null)
            {
                ImGui.SliderInt("X", ref _placeX, _octree.Min.X, _octree.Max.X - 1);
                ImGui.SliderInt("Y", ref _placeY, _octree.Min.Y, _octree.Max.Y - 1);
                ImGui.SliderInt("Z", ref _placeZ, _octree.Min.Z, _octree.Max.Z - 1);

                if (ImGui.Button("Place"))
                {
                    Console.WriteLine($"Insert: {_octree.Insert(new Vector3di(_placeX, _placeY, _placeZ))}");
                    RenderTree();
                }
            }
        }

        ImGui.End();
    }

    private readonly Dictionary<(Vector3di, byte), QuadColors> _nodeColors = new();

    private void RenderTree()
    {
        if (_octree == null)
        {
            return;
        }

        var batch = _treeBatch;

        batch.Clear();

        _octree.Traverse((node, pos, log) =>
        {
            Vector3 min = pos;
            Vector3 max = pos + (1 << log);

            if (node.IsFilled)
            {
                batch.ColoredQuadBox(Matrix4x4.CreateScale((1 << log) * 0.9f) * Matrix4x4.CreateTranslation(
                        (min.X + max.X) / 2f, 
                        (min.Y + max.Y) / 2f, 
                        (min.Z + max.Z) / 2f), 
                    new QuadColors(0.8f, 0.8f, 0.8f, 0.9f)
                );
            }
            else
            {
                var color = _nodeColors.GetOrAdd((pos, log), _ => new QuadColors(Random.Shared.NextVector4(min: 0.25f, max: 1f) with { W = 0.25f }));

                batch.ColoredQuadBoxFrame(
                    min,
                    max,
                    0.1f,
                    color
                );
            }

            return true;
        });

        batch.ColoredQuadBoxFrame(_octree.Min, _octree.Max, 0.1f, new QuadColors(RgbaFloat.White));
    }

    private void RenderHighlight(QuadBatch batch)
    {
        var min = new Vector3(_placeX, _placeY, _placeZ);

        const float margin = 0.1f;

        batch.ColoredQuadBoxFrame(
            min + new Vector3(margin),
            min + new Vector3(1 - margin),
            0.05f,
            new QuadColors(0, 1, 1, 0.25f)
        );
    }

    private static void RenderGizmo(QuadBatch batch)
    {
        const float thickness = 0.25f;
        const float gap = 0.25f;
        const float alpha = 0.5f;

        batch.ColoredQuadBox(
            Matrix4x4.CreateScale(1, thickness, thickness) with
            {
                Translation = new Vector3(thickness / 2f + 0.5f + gap, 0, 0)
            }, 
            new QuadColors(1, 0, 0, alpha)
        );

        batch.ColoredQuadBox(
            Matrix4x4.CreateScale(thickness, 1, thickness) with
            {
                Translation = new Vector3(0, thickness / 2f + 0.5f + gap, 0)
            },
            new QuadColors(0, 1, 0, alpha)
        );

        batch.ColoredQuadBox(
            Matrix4x4.CreateScale(thickness, thickness, 1) with
            {
                Translation = new Vector3(0, 0, thickness / 2f + 0.5f + gap)
            },
            new QuadColors(0, 0, 1, alpha)
        );
    }

    protected override void RenderStack()
    {
        RenderPassMain(RenderGizmo);
        RenderPassMain(RenderHighlight);

        SetCameraTransform(_treeBatch);
        _treeBatch.Submit();
    }
}