using Common;
using Microsoft.Extensions.DependencyInjection;

namespace QuadTreeVisualization;

internal class App : VisualizationApp
{
    protected override void RegisterLayer(ServiceCollection services)
    {
        services.AddSingleton<WorldLayer>();
    }

    protected override void Construct()
    {
        Layers.ConstructLayer<WorldLayer>();
    }
}