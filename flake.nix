{
  description = "frc2023 dev flake";

  inputs.nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.flake-compat = {
    url = "github:edolstra/flake-compat";
    flake = false;
  };

  outputs = {
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };
      in {
        devShell = let
          fhs = pkgs.buildFHSUserEnv {
            name = "frc2023-env";
            targetPkgs = pkgs:
              with pkgs; [
                openjdk11
                gradle
                gcc
                gdb
                astyle

                zlib
                xorg.libX11
                xorg.libXt
                gtk2-x11
                libGL
              ];
          };
        in
          fhs.env;
      }
    );
}
