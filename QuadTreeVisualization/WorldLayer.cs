using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using Veldrid;

namespace QuadTreeVisualization;

internal class WorldLayer : VisualizationLayer
{
    public WorldLayer(GameApplication app, ImGuiLayer imGui) : base(app, imGui) { }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        
    }

    protected override void RenderStack()
    {
        RenderPass(batch =>
        {
            batch.Quad(Vector2.Zero, Vector2.One, RgbaFloat4.Red);
        });
    }
}