using System.Diagnostics;
using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer.Batch;
using ImGuiNET;
using Veldrid;

namespace OctTreeVisualization;

internal sealed class WorldLayer : VisualizationLayer3d
{
    private HashedBitOctree? _octree;

    public WorldLayer(VisualizationApp app, ImGuiLayer imGui) : base(app, imGui)
    {

    }

    private int _recreateLog = 2;
    
    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Visualization"))
        {
            ImGui.SliderInt("Log", ref _recreateLog, 2, 10);

            if (ImGui.Button("Create"))
            {
                _octree = new HashedBitOctree((byte)_recreateLog);
            }
        }

        ImGui.End();
    }

    private void RenderTest(QuadBatch batch)
    {
        if (_octree == null)
        {
            return;
        }

        batch.ColoredQuadBoxFrame(_octree.Min, _octree.Max, 0.1f, new QuadColors(RgbaFloat.White));
    }

    protected override void RenderStack()
    {
        RenderPass(RenderTest);
    }
}