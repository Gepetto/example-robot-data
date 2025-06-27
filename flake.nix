{
  description = "Set of robot URDFs for benchmarking and developed examples";

  inputs = {
    pinocchio.url = "github:nim65s/pinocchio/only-py";
    coal.follows = "pinocchio/coal";
    flake-parts.follows = "pinocchio/flake-parts";
    nixpkgs.follows = "pinocchio/nixpkgs";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        {
          inputs',
          pkgs,
          self',
          system,
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              (final: prev: {
                coal = inputs'.coal.packages.coal-cpp;
                pinocchio = inputs'.pinocchio.packages.pinocchio-cpp;
                pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
                  (python-final: python-prev: {
                    coal = inputs'.coal.packages.coal-py;
                    pinocchio = inputs'.pinocchio.packages.pinocchio-py;
                  })
                ];
              })
            ];
          };
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (p: [
              p.gepetto-gui
              self'.packages.default
            ]);
          };
          devShells.default = pkgs.mkShell {
            inputsFrom = [ self'.packages.default ];
            packages = [
              (pkgs.python3.withPackages (p: [
                p.gepetto-gui
                p.pinocchio
                p.tomlkit # for "make release"
              ]))
            ];
          };
          packages = {
            default = self'.packages.example-robot-data;
            example-robot-data = pkgs.python3Packages.toPythonModule (
              (pkgs.example-robot-data.override { pythonSupport = true; }).overrideAttrs (super: {
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
              })
            );
            example-robot-data-cpp =
              (self'.packages.example-robot-data.override { pythonSupport = false; }).overrideAttrs
                (super: {
                  src = pkgs.lib.fileset.toSource {
                    root = ./.;
                    fileset = pkgs.lib.fileset.unions [
                      ./CMakeLists.txt
                      ./colcon.pkg
                      ./include
                      ./package.xml
                      ./pyproject.toml
                      # ./python
                      ./robots
                      # ./unittest
                    ];
                  };

                });
            example-robot-data-py =
              (self'.packages.example-robot-data.override { pythonSupport = true; }).overrideAttrs
                (super: {
                  cmakeFlags = super.cmakeFlags ++ [ "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON" ];
                  src = pkgs.lib.fileset.toSource {
                    root = ./.;
                    fileset = pkgs.lib.fileset.unions [
                      ./CMakeLists.txt
                      ./colcon.pkg
                      # ./include
                      # ./package.xml
                      ./pyproject.toml
                      ./python
                      # ./robots
                      ./unittest
                    ];
                  };
                  propagatedBuildInputs = super.propagatedBuildInputs ++ [
                    self'.packages.example-robot-data-cpp
                  ];
                });
          };
        };
    };
}
