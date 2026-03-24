{
    description = "enae644-hw03 flake";

    inputs = {
        # nixpkgs
        nixpkgs.url = "github:nixos/nixpkgs/release-25.11";
        # nixpkgs-unstable.url = "github:nixos/nixpkgs/nixos-unstable";

        # flake tools (thanks numtide)
        blueprint = {
            url = "github:numtide/blueprint";
            inputs.nixpkgs.follows = "nixpkgs";
        };
        devshell = {
            url = "github:numtide/devshell";
            inputs.nixpkgs.follows = "nixpkgs";
        };
        treefmt-nix = {
            url = "github:numtide/treefmt-nix";
            inputs.nixpkgs.follows = "nixpkgs";
        };

        # python
        uv2nix = {
            url = "github:pyproject-nix/uv2nix";
            inputs = {
                pyproject-nix.follows = "pyproject-nix";
                nixpkgs.follows = "nixpkgs";
            };
        };
        pyproject-nix = {
            url = "github:pyproject-nix/pyproject.nix";
            inputs = {
                nixpkgs.follows = "nixpkgs";
            };
        };
        pyproject-build-systems = {
            url = "github:pyproject-nix/build-system-pkgs";
            inputs = {
                uv2nix.follows = "uv2nix";
                pyproject-nix.follows = "pyproject-nix";
                nixpkgs.follows = "nixpkgs";
            };
        };

        # typst
        typix = {
            url = "github:loqusion/typix";
            inputs.nixpkgs.follows = "nixpkgs";
        };
    };

    outputs =
        inputs:
        inputs.blueprint {
            inherit inputs;
            prefix = "./nix/";
        };
}
