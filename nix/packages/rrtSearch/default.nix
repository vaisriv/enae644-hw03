{
    pkgs,
    compiler ? "ghc910",
    doCheck ? true,
}:
let
    myHaskellPackages = pkgs.haskell.packages.${compiler}.override {
        overrides = self: _super: rec {
            rrtSearch = self.callCabal2nix "rrtSearch" (./rrtSearch) { };
        };
    };
    drv = myHaskellPackages.callCabal2nix "rrtSearch" (../../../.) { };
in
if doCheck then
    pkgs.haskell.lib.doBenchmark (pkgs.haskell.lib.doCheck drv)
else
    pkgs.haskell.lib.dontCheck drv
