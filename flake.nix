{
  description = "Set of robot URDFs for benchmarking and developed examples";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        {
          pkgs,
          self',
          ...
        }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell {
            inputsFrom = [ self'.packages.default ];
            packages = [ (pkgs.python3.withPackages (p: [p.tomlkit])) ]; # for "make release"
          };
          packages = {
            default = self'.packages.example-robot-data;
            example-robot-data = pkgs.python3Packages.example-robot-data.overrideAttrs (_: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
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
            });
          };
        };
    };
}
