{
    pkgs,
    inputs,
    ...
}:
pkgs.callPackage ./rrtVis.nix {inherit pkgs inputs;}
