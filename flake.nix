{
  description = "An extension of the Flexible Collision Library";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { self, lib, ... }:
      {
        systems = inputs.nixpkgs.lib.systems.flakeExposed;
        flake.overlays = {
          default = final: prev: {
            coal = prev.coal.overrideAttrs {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  ./python
                  ./python-nb
                  ./src
                  ./test
                ];
              };
            };
          };
        };
        perSystem =
          {
            pkgs,
            self',
            system,
            ...
          }:
          {
            _module.args = {
              pkgs = import inputs.nixpkgs {
                inherit system;
                overlays = [ self.overlays.default ];
              };
            };
            apps.default = {
              type = "app";
              program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
            };
            packages = {
              default = self'.packages.coal-full-bp;
              coal-full-bp =
                (pkgs.python3Packages.coal.override { buildStandalone = false; }).overrideAttrs
                  (super: {
                    pname = "coal-full-bp";
                    cmakeFlags = super.cmakeFlags ++ [
                      "-DCOAL_DISABLE_HPP_FCL_WARNINGS=ON"
                      "-DCOAL_PYTHON_NANOBIND=OFF"
                      "-DGENERATE_PYTHON_STUBS=OFF"
                    ];
                  });
              coal-full-nb = self'.packages.coal-full-bp.overrideAttrs (super: {
                pname = "coal-full-nb";
                cmakeFlags = super.cmakeFlags ++ [
                  "-DCOAL_PYTHON_NANOBIND=ON"
                ];
                postPatch = ''
                  substituteInPlace python-nb/CMakeLists.txt --replace-fail \
                    "$""{Python_SITELIB}" \
                    "${pkgs.python3.sitePackages}"
                '';
                propagatedBuildInputs = super.propagatedBuildInputs ++ [
                  pkgs.python3Packages.nanobind
                  pkgs.python3Packages.nanoeigenpy
                ];
                pythonImportsCheck = [ "coal" ]; # hppfcl is broken with nanobind
              });
              libcoal = pkgs.coal;
              coal-nb = pkgs.python3Packages.coal.overrideAttrs (super: {
                pname = "coal-nb";
                cmakeFlags = super.cmakeFlags ++ [
                  "-DCOAL_PYTHON_NANOBIND=ON"
                  "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON"
                ];
                postPatch = ''
                  substituteInPlace python-nb/CMakeLists.txt --replace-fail \
                    "$""{Python_SITELIB}" \
                    "${pkgs.python3.sitePackages}"
                '';
                pythonImportsCheck = [ "coal" ]; # hppfcl is broken with nanobind
                propagatedBuildInputs = super.propagatedBuildInputs ++ [
                  pkgs.python3Packages.nanobind
                  pkgs.python3Packages.nanoeigenpy
                ];
              });
              coal-bp = pkgs.python3Packages.coal.overrideAttrs (super: {
                pname = "coal-bp";
                cmakeFlags = super.cmakeFlags ++ [
                  "-DCOAL_PYTHON_NANOBIND=OFF"
                  "-DGENERATE_PYTHON_STUBS=OFF"
                  "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON"
                ];
              });
            };
          };
      }
    );
}
