# Guide for setting up macOS for computer vision development #

## Switching between compiler version ##
This is necessary when the most recent compiler isn't supported by e.g. CUDA.
- Switch to standard xcode(most recent): `sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer`
- Switch to other xcode(old version): `sudo xcode-select --switch /<path_to_other_xcode_parent_dir>/Xcode.app/Contents/Developer`
- Switch CommandLineTools: `sudo xcode-select --switch /Library/Developer/CommandLineTools`
- Show selected xcode: `xcode-select -print-path`
- Show selected CommandLineTools: `pkgutil --pkg-info=com.apple.pkg.CLTools_Executables
`

## TODO ##
