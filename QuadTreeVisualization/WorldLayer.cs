using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities.Extensions;
using ImGuiNET;
using Veldrid;
using Rectangle = System.Drawing.Rectangle;

namespace QuadTreeVisualization;

internal class WorldLayer : VisualizationLayer
{
    private enum Material
    {
        None,
        A,
        B
    }

    public WorldLayer(GameApplication app, ImGuiLayer imGui) : base(app, imGui) { }

    private ClassicQuadTree<Material>? _tree;

    private int _recreateLog = 1;

    private Material _selectedMat = Material.B;

    protected override unsafe void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Quadtree"))
        {
            ImGui.SliderInt("Log", ref _recreateLog, 1, 16);

            if (ImGui.Button("Recreate"))
            {
                _tree = new ClassicQuadTree<Material>((byte)_recreateLog);
            }

            if (_tree != null)
            {
                if (ImGui.Button("A")) _selectedMat = Material.A;
                if (ImGui.Button("B")) _selectedMat = Material.B;

                ImGui.Text($"Node count: {_tree.NodeCount}");

                var hovered = _tree.Find(MouseI);

                if (hovered != -1)
                {
                    var node = _tree.GetNode(hovered);
                    var data = _tree.GetData(hovered);

                    ImGui.Text($"Hover: {hovered}");
                    ImGui.Text($"Fill: {node.IsFilled}");
                    ImGui.Text($"Children: {node.Children[0]}, {node.Children[1]}, {node.Children[2]}, {node.Children[3]})");
                    ImGui.Text($"Mat: {data}");
                }
            }
        }

        ImGui.End();
    }

    protected override void HandleMouse(MouseEvent e)
    {
        var mouse = MouseI;

        if (_tree != null && _tree.IsWithinBounds(mouse))
        {
            if (e is { Down: false, MouseButton: MouseButton.Left })
            {
                if (_app.Input.IsKeyDown(Key.ShiftLeft))
                {
                    _tree.Remove(mouse);
                }
                else
                {
                    _tree.Insert(mouse, _selectedMat);
                }
            }
        }
    }

    private readonly Dictionary<Rectangle, Vector4> _nodeColors = new();

    private Rectangle NodeRectangle(Vector2di pos, byte log) => new Rectangle(pos.X, pos.Y, 1 << log, 1 << log);

    private Vector4 GetNodeColor(Rectangle nodeRectangle) => _nodeColors.GetOrAdd(nodeRectangle, _ =>
    {
        var random = new Random();
        return random.NextVector4(min: 0.5f) with { W = 1 };
    });

    private void RenderNodes(QuadBatch batch)
    {
        if (_tree == null)
        {
            return;
        }

        _tree.Traverse((int idx, Vector2di position, byte log, in ClassicQuadTreeNode node) =>
        {
            var tl = new Vector2(position.X, position.Y) - new Vector2(0.5f, -0.5f);
            var sz = 1 << log;

            if (!node.IsFilled)
            {
                batch.Quad(
                    tl + new Vector2(sz / 2f, -sz / 2f),
                    new Vector2(sz), 
                    GetNodeColor(NodeRectangle(position, log)) * new Vector4(1, 1, 1, 0.1f));
            }

            return true;
        });
    }

    private void RenderSilkscreen(QuadBatch batch)
    {
        if (_tree == null)
        {
            return;
        }

        _tree.Traverse((int idx, Vector2di position, byte log, in ClassicQuadTreeNode node) =>
        {
            var tl = new Vector2(position.X, position.Y) - new Vector2(0.5f, -0.5f);
            var sz = 1 << log;

            if (node.IsFilled)
            {
                var data = _tree.GetData(idx);

                if (data != Material.None)
                {
                    batch.Quad(tl + new Vector2(sz / 2f, -sz / 2f), new Vector2(0.2f), data switch
                    {
                        Material.A => RgbaFloat4.Red,
                        Material.B => RgbaFloat4.Cyan,
                        _ => throw new ArgumentOutOfRangeException()
                    });
                }
            }
            else
            {
                var color = GetNodeColor(NodeRectangle(position, log)) * new Vector4(1, 1, 1, 0.8f);

                var dx = new Vector2(sz, 0);
                var dy = new Vector2(0, sz);
                var dx2 = dx / 2f;
                var dy2 = dy / 2f;

                batch.Line(tl, tl + dx, color, 0.2f);
                batch.Line(tl - dy, tl - dy + dx, color, 0.2f);
                batch.Line(tl, tl - dy, color, 0.2f);
                batch.Line(tl + dx, tl + dx - dy, color, 0.2f);
                batch.Line(tl - dy2, tl + dx - dy2, color, 0.2f);
                batch.Line(tl + dx2, tl + dx2 - dy, color, 0.2f);
            }

            return true;
        });
    }

    protected override void RenderStack()
    {
        RenderPass(RenderNodes);
        RenderPass(RenderSilkscreen);
    }
}