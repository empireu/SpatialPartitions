using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer.Batch;
using Veldrid;

namespace OctTreeVisualization;

internal sealed class WorldLayer : VisualizationLayer3d
{
    public WorldLayer(GameApplication app, ImGuiLayer imGui) : base(app, imGui)
    {
    }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
            
    }

    private void RenderTest(QuadBatch batch)
    {
        batch.ColoredQuadCube(Matrix4x4.CreateTranslation(0f, 0f, -10f), QuadColors.FromZOrder(RgbaFloat.Red, RgbaFloat.Green, RgbaFloat.Blue, RgbaFloat.Pink));
        
        batch.ColoredQuadCube(
            Matrix4x4.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 4f) * 
            Matrix4x4.CreateScale(2.0f) * 
            Matrix4x4.CreateTranslation(0f, 0f, -5f), 
            QuadColors.FromZOrder(RgbaFloat.Red, RgbaFloat.Green, RgbaFloat.Blue, RgbaFloat.Pink)
        );
    }

    protected override void RenderStack()
    {
        RenderPass(RenderTest);
    }
}