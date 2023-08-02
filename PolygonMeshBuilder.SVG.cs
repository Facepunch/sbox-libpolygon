using System;
using Sandbox.Utility.Svg;

namespace Sandbox.Polygons;

/// <summary>
/// Options for <see cref="PolygonMeshBuilder.AddSvg"/>.
/// </summary>
public class AddSvgOptions
{
    public static AddSvgOptions Default { get; } = new();

    /// <summary>
    /// If true, any unsupported path types will throw an exception. Defaults to false.
    /// </summary>
    public bool ThrowIfNotSupported { get; set; }

    /// <summary>
    /// Maximum distance between vertices on curved paths. Defaults to 1.
    /// </summary>
    public float CurveResolution { get; set; } = 1f;
}

partial class PolygonMeshBuilder
{
    /// <summary>
    /// Add all supported paths from the given SVG document.
    /// </summary>
    /// <param name="contents">SVG document contents.</param>
    /// <param name="options">Options for generating vertices from paths.</param>
    public void AddSvg( string contents, AddSvgOptions options = null )
    {
        var svg = SvgDocument.FromString( contents );

        foreach ( var path in svg.Paths )
        {
            AddPath( path, options );
        }
    }

    private static void ThrowNotSupported( AddSvgOptions options, string message )
    {
        if ( !options.ThrowIfNotSupported )
        {
            return;
        }

        throw new NotImplementedException( $"SVG path element not supported: {message}" );
    }

    /// <summary>
    /// Add an individual path from an SVG document, if supported.
    /// </summary>
    /// <param name="path">SVG path element.</param>
    /// <param name="options">Options for generating vertices from paths.</param>
    public void AddPath( SvgPath path, AddSvgOptions options = null )
    {
        options ??= AddSvgOptions.Default;

        if ( path.IsEmpty )
        {
            return;
        }

        foreach ( var cmd in path.Commands )
        {
            switch ( cmd )
            {
                case AddPolyPathCommand addPolyPathCommand:
                    AddPolyPath( addPolyPathCommand, options );
                    break;
                default:
                    ThrowNotSupported( options, $"{cmd.GetType()}" );
                    break;
            }
        }
    }

    private void AddPolyPath( AddPolyPathCommand cmd, AddSvgOptions options )
    {
        if ( !cmd.Close )
        {
            ThrowNotSupported( options, $"{cmd.GetType()} with Close = False" );
            return;
        }

        AddEdgeLoop( cmd.Points, 0, cmd.Points.Count );
    }
}
