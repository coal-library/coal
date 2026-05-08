{
  description = "An extension of the Flexible Collision Library";

  inputs = {
    gepetto.url = "github:gepetto/nix";

    # eigenpy v3.12.0 does not handle eigen v5, so we need devel for now
    eigenpy.url = "github:stack-of-tasks/eigenpy";
    eigenpy.inputs.gepetto.follows = "gepetto";
  };

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overlays = [ inputs.eigenpy.overlays.flakoboros ];
        extraDevPyPackages = [ "coal" ];
        overrideAttrs.coal =
          { drv-prev, ... }:
          {
            cmakeFlags = drv-prev.cmakeFlags ++ [
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
          };
        extends = {
          inherit (inputs.eigenpy.overlays) eigen5;
          full = _final: prev: {
            pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
              (_python-final: python-prev: {
                coal = python-prev.coal.override { buildStandalone = false; };
              })
            ];
          };
          nanobind = _final: prev: {
            pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
              (python-final: python-prev: {
                coal = python-prev.coal.overrideAttrs (super: {
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
      }
    );
}
