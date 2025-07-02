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
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.coal-bp;
            coal-bp = pkgs.python3Packages.coal.overrideAttrs (super: {
              pname = "coal-bp";
              cmakeFlags = super.cmakeFlags ++ [
                "-DCOAL_PYTHON_NANOBIND=OFF"
                "-DGENERATE_PYTHON_STUBS=OFF"
              ];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  ./python
                  # ./python-nb
                  ./src
                  ./test
                ];
              };
            });
            coal-nb = pkgs.python3Packages.coal.overrideAttrs (super: {
              pname = "coal-nb";
              cmakeFlags = super.cmakeFlags ++ [ "-DCOAL_PYTHON_NANOBIND=ON" ];
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
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  # ./python
                  ./python-nb
                  ./src
                  ./test
                ];
              };
            });
          };
        };
    };
}
