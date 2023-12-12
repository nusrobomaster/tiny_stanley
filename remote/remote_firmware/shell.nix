{ pkgs ? import <nixpkgs> {} }:
  pkgs.mkShell {
    nativeBuildInputs = with pkgs.buildPackages; [ 
      ccls
      bear

      openocd
      gdb

      gnumake
      gcc-arm-embedded
    ];

  shellHook = ''
    bin_loc=build/''${PWD##*/}.elf
    alias clean="rm -r build/*"
    alias build="bear -- make -j8"
    alias conup="openocd -f openocd.cfg"
    alias flash='openocd -f openocd.cfg -c "program ''${bin_loc} verify reset exit"'
    alias upload="build && flash"
    alias debug='build && gdb ''${bin_loc} -ex "target extended-remote localhost:3333"'
  '';
}
