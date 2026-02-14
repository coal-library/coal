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
            coal = prev.coal.overrideAttrs (super: {
              cmakeFlags = super.cmakeFlags ++ [
                "-DCOAL_DISABLE_HPP_FCL_WARNINGS=ON"
                "-DGENERATE_PYTHON_STUBS=OFF"
              ];
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
            });

            pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
              (python-final: python-prev: {
                coal = python-prev.coal.overrideAttrs (super: {
                  pname = "coal-bp";
                  cmakeFlags = super.cmakeFlags ++ [
                    "-DCOAL_PYTHON_NANOBIND=OFF"
                  ];
                });

                coal-nb = python-final.coal.overrideAttrs (super: {
                  pname = "coal-nb";
                  cmakeFlags = super.cmakeFlags ++ [
                    "-DCOAL_PYTHON_NANOBIND=ON"
                  ];
                  postPatch = ''
                    substituteInPlace python-nb/CMakeLists.txt --replace-fail \
                      "$""{Python_SITELIB}" \
                      "${python-final.python.sitePackages}"
                  '';
                  propagatedBuildInputs = super.propagatedBuildInputs ++ [
                    python-final.nanobind
                    python-final.nanoeigenpy
                  ];
                  pythonImportsCheck = [ "coal" ]; # hppfcl is broken with nanobind
                });
              })
            ];
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
              libcoal = pkgs.coal;
              coal-bp = pkgs.python3Packages.coal;
              coal-nb = pkgs.python3Packages.coal-nb;
              coal-full-bp = (pkgs.python3Packages.coal.override { buildStandalone = false; }).overrideAttrs {
                pname = "coal-full-bp";
              };
              coal-full-nb = (pkgs.python3Packages.coal-nb.override { buildStandalone = false; }).overrideAttrs {
                pname = "coal-full-nb";
              };
            };
          };
      }
    );
}
