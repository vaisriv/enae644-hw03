{pkgs}:
pkgs.stdenv.mkDerivation {
    pname = "rrtSearch";
    version = "0.1.0.0";
    src = ./.;
    isLibrary = false;
    isExecutable = true;
    mainProgram = "rrtSearch";

    meta = {
        homepage = "https://github.com/vaisriv/enae644-hw02";
        changelog = "https://github.com/vaisriv/enae644-hw02/blob/main/CHANGELOG.md";

        license = pkgs.lib.licenses.mit;
        maintainers = with pkgs.lib.maintainers; [vaisriv];
    };
}
