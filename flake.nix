{
  description = "Set of robot URDFs for benchmarking and developed examples";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          { gepetto-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default = _final: prev: {
          # Don't override pkgs.example-robot-data, or it would lead to a pinocchio rebuild
          pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
            (_python-final: python-prev: {

              example-robot-data =
                (python-prev.example-robot-data.override { standalone = false; }).overrideAttrs
                  {
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
            })
          ];
        };
        perSystem =
          { pkgs, self', ... }:
          {
            apps.default = {
              type = "app";
              program = pkgs.python3.withPackages (p: [
                p.gepetto-gui
                p.meshcat
                p.viser
                self'.packages.default
              ]);
            };
            devShells.default = pkgs.mkShell {
              inputsFrom = [ self'.packages.default ];
              packages = [
                (pkgs.python3.withPackages (p: [
                  p.gepetto-gui
                  p.meshcat
                  p.pinocchio
                  p.viser
                ]))
              ];
            };
            packages = {
              default = self'.packages.example-robot-data;
              example-robot-data = pkgs.python3Packages.example-robot-data;
            };
          };
      }
    );
}
