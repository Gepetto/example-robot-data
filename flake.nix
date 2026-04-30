{
  description = "Set of robot URDFs for benchmarking and developed examples";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraPyPackages = [
          "meshcat"
          "pinocchio"
          "viser"
        ];
        # don't trigger a pinocchio rebuild by overriding example-robot-data
        pyOverrides.example-robot-data = {
          buildStandalone = false;
        };
        pyOverrideAttrs.example-robot-data = {
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
      }
    );
}
