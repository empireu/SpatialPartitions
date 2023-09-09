using System.Numerics;
using Common;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities;
using GameFramework.Utilities.Extensions;
using ImGuiNET;
using Veldrid;

namespace OctTreeVisualization;

internal sealed class WorldLayer : VisualizationLayer3d
{
    private readonly QuadBatch _proxyBatch;
    private readonly QuadBatch _voxelBatch;
    private BitOctree? _octree;

    private int _recreateLog = 2;
    private int _placeX;
    private int _placeY;
    private int _placeZ;

    private int _lastRenderedVersion = -1;
    
    private float _f1 = 0.02f;
    private float _a1 = 10f;

    private ulong _selected;

    private bool _renderTerrain = true;

    public WorldLayer(VisualizationApp app, ImGuiLayer imGui) : base(app, imGui)
    {
        _proxyBatch = GetBatch();
        _voxelBatch = GetBatch();

        CameraController.MoveSpeed = 5f;
    }

    protected override void UpdateBatchPipelines(OutputDescription outputDesc)
    {
        _proxyBatch.UpdatePipelines(
            outputDescription: outputDesc, 
            blendState: BlendStateDescription.SingleAdditiveBlend, 
            depthStencilState: DepthStencilStateDescription.Disabled
        );

        _voxelBatch.UpdatePipelines(
            outputDescription: outputDesc,
            blendState: BlendStateDescription.SingleAlphaBlend,
            depthStencilState: DepthStencilStateDescription.DepthOnlyLessEqual
        );
    }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Octree"))
        {
            ImGui.SliderInt("Log", ref _recreateLog, 1, 21);

            if (ImGui.Button("Create"))
            {
                _octree = new BitOctree((byte)_recreateLog);
                _lastRenderedVersion = -1;
            }

            if (_octree != null)
            {
                if (ImGui.Button("Clear"))
                {
                    _octree.Clear();
                }

                if (ImGui.Button("Clear (fill)"))
                {
                    _octree.Clear(true);
                }

                ImGui.Text($"Nodes: {_octree.NodeCount}");
                ImGui.Text($"Contains: {_octree.Contains(new Vector3di(_placeX, _placeY, _placeZ))}");

                ImGui.SliderInt("X", ref _placeX, _octree.Min.X, _octree.Max.X - 1);
                ImGui.SliderInt("Y", ref _placeY, _octree.Min.Y, _octree.Max.Y - 1);
                ImGui.SliderInt("Z", ref _placeZ, _octree.Min.Z, _octree.Max.Z - 1);

                ImGui.Text($"Closest Code: {_selected}");
                ImGui.Text($"Closest Pos: {_octree.DecodePosition(_selected)}");
                ImGui.Text($"Closest Log: {_octree.DecodeLog(_selected)}");

                if (ImGui.IsKeyReleased(ImGuiKey.RightArrow))
                {
                    _placeX = Math.Clamp(_placeX + 1, _octree.Min.X, _octree.Max.X - 1);
                }

                if (ImGui.IsKeyReleased(ImGuiKey.LeftArrow))
                {
                    _placeX = Math.Clamp(_placeX - 1, _octree.Min.X, _octree.Max.X - 1);
                }

                if (ImGui.IsKeyReleased(ImGuiKey.DownArrow))
                {
                    _placeZ = Math.Clamp(_placeZ + 1, _octree.Min.Z, _octree.Max.Z - 1);
                }

                if (ImGui.IsKeyReleased(ImGuiKey.UpArrow))
                {
                    _placeZ = Math.Clamp(_placeZ - 1, _octree.Min.Z, _octree.Max.Z - 1);
                }

                if (ImGui.IsKeyReleased(ImGuiKey.PageUp))
                {
                    _placeY = Math.Clamp(_placeY + 1, _octree.Min.Y, _octree.Max.Y - 1);
                }

                if (ImGui.IsKeyReleased(ImGuiKey.PageDown))
                {
                    _placeY = Math.Clamp(_placeY - 1, _octree.Min.Y, _octree.Max.Y - 1);
                }

                if (ImGui.Button("Place") || ImGui.IsKeyReleased(ImGuiKey.Enter))
                {
                    Console.WriteLine($"Insert: {_octree.Insert(new Vector3di(_placeX, _placeY, _placeZ))}");
                }

                if (ImGui.Button("Remove") || ImGui.IsKeyReleased(ImGuiKey.Backspace))
                {
                    Console.WriteLine($"Remove: {_octree.Remove(new Vector3di(_placeX, _placeY, _placeZ))}");
                }

                ImGui.Separator();

                ImGui.InputFloat("F1", ref _f1);
                ImGui.InputFloat("A1", ref _a1);

                if (ImGui.Button("Randomize"))
                {
                    RandomGeneration();
                }

                if (ImGui.Button("Test range"))
                {
                    var min = new Vector3di(
                        Random.Shared.Next(0, _octree.EdgeSize - 1),
                        Random.Shared.Next(0, _octree.EdgeSize - 1),
                        Random.Shared.Next(0, _octree.EdgeSize - 1)
                    );

                    var max = new Vector3di(
                        Random.Shared.Next(min.X + 1, _octree.EdgeSize),
                        Random.Shared.Next(min.Y + 1, _octree.EdgeSize),
                        Random.Shared.Next(min.Z + 1, _octree.EdgeSize)
                    );

                    Console.WriteLine($"Range: {min} - {max}");

                    var a = new Grid3d<bool>(max - min);
                    var b = new Grid3d<bool>(max - min);

                    var rangeTime = Measurements.MeasureSecondsD(() =>
                    {
                        _octree.ReadRange(min, a);
                    });

                    var results = 0;

                    var naiveTime = Measurements.MeasureSecondsD(() =>
                    {
                        for (var z = 0; z < b.Size.Z; z++)
                        {
                            for (var y = 0; y < b.Size.Y; y++)
                            {
                                for (var x = 0; x < b.Size.X; x++)
                                {
                                    var res = _octree.Contains(new Vector3di(x, y, z) + min);
                                
                                    b[x, y, z] = res;

                                    if (res)
                                    {
                                        ++results;
                                    }
                                }
                            }
                        }
                    });

                    Assert.IsTrue(a.Storage.SequenceEqual(b.Storage), "Test failed");
                    Console.WriteLine($"{results} hits");
                    Console.WriteLine($"Range: {rangeTime * 1000:F3}ms, Naive: {naiveTime * 1000:F3}ms");
                }

                ImGui.Separator();

                ImGui.Checkbox("Render Terrain", ref _renderTerrain);
            }
        }

        ImGui.End();
    }

    private void RandomGeneration()
    {
        if (_octree == null)
        {
            return;
        }

        for (var x = 0; x < _octree.EdgeSize; x++)
        {
            for (var z = 0; z < _octree.EdgeSize; z++)
            {
                var height = (int)(_octree.EdgeSize / 2f + _a1 * Math.Sin(x * _f1));

                for (int y = 0; y < height; y++)
                {
                    var tile = new Vector3di(x, y, z);
                 
                    if (_octree.IsWithinBounds(tile))
                    {
                        _octree.Insert(tile);
                    }
                }

            }
        }
    }

    private readonly Dictionary<(Vector3di, byte), Vector4> _nodeColors = new();

    private void SubmitTree()
    {
        if (_octree == null)
        {
            return;
        }

        SetCameraTransform(_proxyBatch);
        SetCameraTransform(_voxelBatch);

        if (_lastRenderedVersion != _octree.Version)
        {
            _lastRenderedVersion = _octree.Version;

            _proxyBatch.Clear();
            _voxelBatch.Clear();

            _octree.Traverse((node, lc, pos, log) =>
            {
                Vector3 min = pos;
                Vector3 max = pos + (1 << log);

                var color = _nodeColors.GetOrAdd((pos, log), _ => Random.Shared.NextVector4(min: 0.25f, max: 0.8f));

                if (node.IsFilled)
                {
                    _voxelBatch.ColoredQuadBox(Matrix4x4.CreateScale((1 << log) - 0.05f) * Matrix4x4.CreateTranslation(
                            (min.X + max.X) / 2f,
                            (min.Y + max.Y) / 2f,
                            (min.Z + max.Z) / 2f),
                        new QuadColors(color with { W = 1 })
                    );
                }
                else
                {
                    _proxyBatch.ColoredQuadBoxFrame(
                        min,
                        max,
                        0.1f,
                        new QuadColors(color with { W = 0.25f })
                    );
                }

                return true;
            });

            _proxyBatch.ColoredQuadBoxFrame(_octree.Min, _octree.Max, 0.1f, new QuadColors(RgbaFloat.White));
        }

        _proxyBatch.Submit();
        _voxelBatch.Submit();
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
  
    private void RenderHighlight(QuadBatch batch)
    {
        var min = new Vector3(_placeX, _placeY, _placeZ);

        const float margin = -0.025f;

        batch.ColoredQuadBoxFrame(
            min + new Vector3(margin),
            min + new Vector3(1 - margin),
            0.05f,
            new QuadColors(0, 0.5f, 0.8f, 1)
        );
    }

    private void RenderSelections(QuadBatch batch)
    {
        if (_octree == null)
        {
            return;
        }

        var queryPos = new Vector3di(_placeX, _placeY, _placeZ);

        _octree.GetClosestVoxel(queryPos)?.Also(result =>
        {
            BoundingBox3di bounds = result.Bounds;

            batch.ColoredQuadBoxFrame(bounds.Min, bounds.Max, 0.05f, new QuadColors(0.9f, 0.2f, 0.1f, 0.5f));

            _selected = result.LocationCode;
        });

        _octree.GetClosestEmptyRegion(queryPos)?.Also(result =>
        {
            BoundingBox3di bounds = result;

            batch.ColoredQuadBox(bounds.Min, bounds.Max, new QuadColors(0.1f, 0.1f, 0.9f, 0.2f));
        });
    }

    private void RenderFrontier(QuadBatch batch)
    {
        if (_octree == null)
        {
            return;
        }

        foreach (var posD in _octree.EnumerateFrontierCells())
        {
            Vector3 pos = posD;

            batch.ColoredQuadBox(
                pos - new Vector3(0.25f),
                pos + new Vector3(0.25f),
                new QuadColors(1, 1, 1, 1)
            );
        }
    }

    protected override void RenderStack()
    {
        if (_renderTerrain)
        {
            SubmitTree();
        }

        RenderPassMain(RenderGizmo);
        //RenderPassMain(RenderSelections);
        RenderPassMain(RenderHighlight);
        RenderPassMain(RenderFrontier);
    }
}