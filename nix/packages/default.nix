{
    pkgs,
    compiler ? "ghc910",
}: let
    myHaskellPackages = pkgs.haskell.packages.${compiler}.override {
        overrides = self: _super: rec {
            enae644-hw02 = self.callCabal2nix "enae644-hw02" (./enae644-hw02) {};
        };
    };
in
    myHaskellPackages.callCabal2nix "enae644-hw02" (../../.) {}
