{
    pkgs,
    inputs,
    ...
}:
let
    workspace = inputs.uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ../../../.; };
    overlay = workspace.mkPyprojectOverlay {
        sourcePreference = "wheel";
    };
    pythonBase = pkgs.callPackage inputs.pyproject-nix.build.packages {
        python = pkgs.python3;
    };
    pythonSets = pythonBase.overrideScope (
        pkgs.lib.composeManyExtensions [
            inputs.pyproject-build-systems.overlays.wheel
            overlay
        ]
    );
in
pythonSets.mkVirtualEnv "rrtVis" workspace.deps.default
