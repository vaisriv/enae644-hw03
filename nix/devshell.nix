{
    pkgs,
    perSystem,
    ...
}:
perSystem.devshell.mkShell {
    name = "enae644-hw03 devshell";
    motd = ''
        {141}📚 hw{reset} devshell
        $(type -p menu &>/dev/null && menu)
    '';

    commands = [ ];

    packages = with pkgs; [
        # haskell
        ghc
        cabal-install
        haskell-language-server

        # python
        python3
        uv
        ty

        # typst
        tinymist
    ];
}
