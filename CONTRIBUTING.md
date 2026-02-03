# Contributing

Issues, discussions, and forks may be created at the contributor’s discretion.  
Pull requests are not expected to be accepted.


## Repository structure

```
.
└── docs                    # Protocol and firmware documentation
│   └── usb2canfdv1.wiki    # Git subtree (external wiki repo)
├── test                    # Test scripts for system test
│   └── slcan-tester        # Git subtree (external repo)
└── usb2canfdv1-fw          # Firmware source code
    ├── Libs                # External libraries
    │   └── printf          # Git submodule (external repo)
    └── ...
```

The commands below sync changes between this repository and subtrees
([wiki](https://github.com/Nakakiyo092/usb2canfdv1/wiki) and [slcan-tester](https://github.com/Nakakiyo092/slcan-tester)).  
It would be best practice to sync at release time.

```
git subtree pull --prefix=doc doctree main --squash
git subtree push --prefix=doc doctree main
git subtree pull --prefix=test testtree main --squash
git subtree push --prefix=test testtree main
```


## Backward compatibility

- Compatibility with LAWICEL CAN ASCII protocol is mandatory.
- Compatibility with the SLCAN protocol, including older versions in this repository, is not required.
- That said, breaking changes should be reserved for major releases.


## Test policy

- Review individual test cases as needed before submitting a pull request.
- Confirm all standard test cases before releasing firmware.
- Visually inspect the pre-release checklist prior to release.
