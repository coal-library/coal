{
  description = "An extension of the Flexible Collision Library";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          packages = {
            default = self'.packages.coal;
            coal = pkgs.python3Packages.toPythonModule (
              (pkgs.coal.override { pythonSupport = true; }).overrideAttrs (super: {
                cmakeFlags = super.cmakeFlags ++ [ "-DCOAL_DISABLE_HPP_FCL_WARNINGS=ON" ];
                src = pkgs.lib.fileset.toSource {
                  root = ./.;
                  fileset = pkgs.lib.fileset.unions [
                    ./CMakeLists.txt
                    ./doc
                    ./hpp-fclConfig.cmake
                    ./include
                    ./package.xml
                    ./python
                    ./src
                    ./test
                  ];
                };
              })
            );
            coal-cpp = (self'.packages.coal.override { pythonSupport = false; }).overrideAttrs (super: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  # ./python
                  ./src
                  ./test
                ];
              };
            });
            coal-py = (self'.packages.coal.override { pythonSupport = true; }).overrideAttrs (super: {
              cmakeFlags = super.cmakeFlags ++ [ "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON" ];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  ./python
                  # ./src
                  ./test
                ];
              };
              propagatedBuildInputs = super.propagatedBuildInputs ++ [
                self'.packages.coal-cpp
              ];
            });
          };
        };
    };
}
