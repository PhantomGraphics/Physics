param(
    [ValidateRange(64, 2048)]
    [int]$Size = 512,
    [string]$OutputDir = $PSScriptRoot
)

$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$source = @'
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;

public static class IndoorCubeMapGenerator
{
    private struct V3
    {
        public double X, Y, Z;
        public V3(double x, double y, double z)
        {
            double length = Math.Sqrt(x*x + y*y + z*z);
            X = x / length; Y = y / length; Z = z / length;
        }
    }

    private static V3 Direction(int face, double u, double v)
    {
        switch (face) {
        case 0: return new V3( 1, -v, -u); // +X right
        case 1: return new V3(-1, -v,  u); // -X left
        case 2: return new V3( u,  1,  v); // +Y top
        case 3: return new V3( u, -1, -v); // -Y bottom
        case 4: return new V3( u, -v,  1); // +Z front
        default:return new V3(-u, -v, -1); // -Z back
        }
    }

    private static byte Clamp(double value)
    {
        return (byte)Math.Max(0, Math.Min(255, (int)Math.Round(value)));
    }

    private static Color Trace(V3 d)
    {
        const double halfWidth = 6.0, halfHeight = 3.0, halfDepth = 6.0;
        double tx = halfWidth / Math.Max(Math.Abs(d.X), 1e-9);
        double ty = halfHeight / Math.Max(Math.Abs(d.Y), 1e-9);
        double tz = halfDepth / Math.Max(Math.Abs(d.Z), 1e-9);
        double t = Math.Min(tx, Math.Min(ty, tz));
        double x = d.X*t, y = d.Y*t, z = d.Z*t;

        double r, g, b;
        if (ty <= tx && ty <= tz && d.Y > 0) {
            // Warm white ceiling with two broad luminous panels.
            r = 205; g = 207; b = 202;
            bool panel = Math.Abs(x) < 2.2 &&
                         (Math.Abs(z - 2.4) < 0.75 || Math.Abs(z + 2.4) < 0.75);
            if (panel) { r = 255; g = 244; b = 218; }
        } else if (ty <= tx && ty <= tz) {
            // Dark satin floor with a restrained large-tile pattern.
            int tile = ((int)Math.Floor((x + 6.0) / 1.5) +
                        (int)Math.Floor((z + 6.0) / 1.5)) & 1;
            r = 38 + tile*5; g = 45 + tile*5; b = 52 + tile*6;
        } else if (tz <= tx && d.Z > 0) {
            // Cool daylight window on the front wall.
            bool window = Math.Abs(x) < 3.2 && y > -1.6 && y < 1.9;
            if (window) {
                double vertical = (y + 1.6) / 3.5;
                r = 132 + 62*vertical; g = 184 + 52*vertical; b = 224 + 28*vertical;
                if (Math.Abs(x) < 0.055 || Math.Abs(y - 0.15) < 0.045) {
                    r = 54; g = 64; b = 72;
                }
            } else { r = 176; g = 180; b = 178; }
        } else if (tz <= tx) {
            // Muted terracotta accent wall behind the camera.
            r = 142; g = 82; b = 62;
            if (y < -2.15) { r = 54; g = 48; b = 46; }
        } else {
            // Quiet neutral side walls with a dark skirting board.
            r = 184; g = 187; b = 183;
            if (y < -2.35) { r = 48; g = 52; b = 55; }
        }

        // Soft ambient falloff keeps corners readable without looking flat.
        double corner = Math.Min(1.0, (Math.Abs(x)/halfWidth +
                                      Math.Abs(y)/halfHeight +
                                      Math.Abs(z)/halfDepth) / 2.25);
        double shade = 1.0 - 0.16*corner;
        return Color.FromArgb(255, Clamp(r*shade), Clamp(g*shade), Clamp(b*shade));
    }

    public static void Generate(string outputDir, int size)
    {
        string[] names = { "right.png", "left.png", "top.png", "bottom.png",
                           "front.png", "back.png" };
        Directory.CreateDirectory(outputDir);
        for (int face = 0; face < 6; ++face) {
            using (var image = new Bitmap(size, size, PixelFormat.Format32bppArgb)) {
                for (int y = 0; y < size; ++y) {
                    for (int x = 0; x < size; ++x) {
                        double u = 2.0*(x + 0.5)/size - 1.0;
                        double v = 2.0*(y + 0.5)/size - 1.0;
                        image.SetPixel(x, y, Trace(Direction(face, u, v)));
                    }
                }
                image.Save(Path.Combine(outputDir, names[face]), ImageFormat.Png);
            }
        }
    }
}
'@

if (-not ('IndoorCubeMapGenerator' -as [type])) {
    Add-Type -TypeDefinition $source -ReferencedAssemblies System.Drawing
}

$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDir)
[IndoorCubeMapGenerator]::Generate($resolvedOutput, $Size)
Write-Host "Generated indoor SSFR cubemap ($Size x $Size) in $resolvedOutput"
