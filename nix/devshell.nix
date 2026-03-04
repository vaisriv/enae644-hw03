{
    pkgs,
    perSystem,
    ...
}:
perSystem.devshell.mkShell {
    name = "enae644-hw02 devshell";
    motd = ''
        {141}📚 hw{reset} devshell
        $(type -p menu &>/dev/null && menu)
    '';

    commands = [];

    packages = with pkgs; [
        # python
        python3
        uv
        ty

        # haskell
        ghc
        cabal-install
        haskell-language-server
    ];
}
