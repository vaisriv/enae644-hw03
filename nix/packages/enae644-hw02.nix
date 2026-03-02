{pkgs}:
pkgs.stdenv.mkDerivation {
    pname = "enae644-hw02";
    version = "0.1.0.0";
    src = ./.;
    isLibrary = false;
    isExecutable = true;
    mainProgram = "enae644-hw02";

    meta = {
        homepage = "https://github.com/vaisriv/enae644-hw02";
        changelog = "https://github.com/vaisriv/enae644-hw02/blob/main/CHANGELOG.md";

        license = pkgs.lib.licenses.mit;
        maintainers = with pkgs.lib.maintainers; [vaisriv];
    };
}
