{
    pkgs,
    inputs,
    ...
}:
inputs.treefmt-nix.lib.mkWrapper pkgs {
    projectRootFile = "flake.nix";

    # nix
    programs = {
        deadnix.enable = true;
        nixfmt = {
            enable = true;
            indent = 4;
        };
    };

    # haskell
    programs = {
        ormolu.enable = true;
    };

    # python
    programs = {
        ruff-check.enable = true;
        ruff-format.enable = true;
    };

    # json/markdown
    programs.prettier = {
        enable = true;
        settings = {
            tabWidth = 4;
        };
    };

    # typst
    programs.typstyle.enable = true;
    settings.formatter = {
        typstyle = {
            options = [
                "--tab-width"
                "4"
            ];
        };
    };
}
