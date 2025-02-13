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
            default = self'.packages.coal-full-bp;
            coal-full-bp = pkgs.python3Packages.toPythonModule (
              (pkgs.coal.override { pythonSupport = true; }).overrideAttrs (super: {
                pname = "coal-full-bp";
                cmakeFlags = super.cmakeFlags ++ [
                  "-DCOAL_DISABLE_HPP_FCL_WARNINGS=ON"
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
              })
            );
            coal-full-nb =
              (self'.packages.coal-full-bp.override { pythonSupport = true; }).overrideAttrs
                (super: {
                  pname = "coal-full-nb";
                  cmakeFlags = super.cmakeFlags ++ [
                    "-DCOAL_PYTHON_NANOBIND=ON"
                  ];
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
            coal-cpp = (self'.packages.coal-full-bp.override { pythonSupport = false; }).overrideAttrs (super: {
              pname = "coal-cpp";
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./hpp-fclConfig.cmake
                  ./include
                  ./package.xml
                  # ./python
                  # ./python-nb
                  ./src
                  ./test
                ];
              };
            });
            coal-nb = (self'.packages.coal-full-bp.override { pythonSupport = true; }).overrideAttrs (super: {
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
                  # ./src
                  ./test
                ];
              };
              pythonImportsCheck = [ "coal" ]; # hppfcl is broken with nanobind
              propagatedBuildInputs = super.propagatedBuildInputs ++ [
                self'.packages.coal-cpp
                pkgs.python3Packages.nanobind
                pkgs.python3Packages.nanoeigenpy
              ];
            });
            coal-bp = (self'.packages.coal-full-bp.override { pythonSupport = true; }).overrideAttrs (super: {
              pname = "coal-bp";
              cmakeFlags = super.cmakeFlags ++ [
                "-DCOAL_PYTHON_NANOBIND=OFF"
                "-DGENERATE_PYTHON_STUBS=OFF"
                "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON"
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
