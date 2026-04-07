{
  description = "Set of robot URDFs for benchmarking and developed examples";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros = {
              extraPyPackages = [
                "meshcat"
                "pinocchio"
                "viser"
              ];
              # don't trigger a pinocchio rebuild by overriding example-robot-data
              pyOverrides.example-robot-data = _: _: { buildStandalone = false; };
              pyOverrideAttrs.example-robot-data = _: _: {
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./CMakeLists.txt
                    ./colcon.pkg
                    ./include
                    ./package.xml
                    ./pyproject.toml
                    ./python
                    ./robots
                    ./unittest
                  ];
                };
              };
            };
          }
        ];
      }
    );
}
