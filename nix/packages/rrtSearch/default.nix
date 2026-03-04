{
    pkgs,
    compiler ? "ghc910",
}: let
    myHaskellPackages = pkgs.haskell.packages.${compiler}.override {
        overrides = self: _super: rec {
            rrtSearch = self.callCabal2nix "rrtSearch" (./rrtSearch) {};
        };
    };
in
    myHaskellPackages.callCabal2nix "rrtSearch" (../../../.) {}
