# Contributing

Issues, discussions, and forks may be created at the contributor’s discretion.  
Pull requests are not expected to be accepted.


## Repository structure

```
.
├── benchmark/              # Result of performance tests
├── doc/                    # Protocol and firmware documentation
│   └── usb2canfdv1.wiki    # Git subtree (external wiki repo)
├── test/                   # Test scripts for system test
│   └── slcan-tester        # Git subtree (external repo)
└── usb2canfdv1-fw/         # Firmware source code
```

The commands below sync changes between this repository and the subtrees
([wiki](https://github.com/Nakakiyo092/usb2canfdv1/wiki) and [slcan-tester](https://github.com/Nakakiyo092/slcan-tester)).  
It would be a good practice to sync at release time.

```
git subtree pull --prefix=doc doctree master --squash
git subtree push --prefix=doc doctree master --squash
git subtree pull --prefix=test testtree merge
git subtree push --prefix=test testtree merge
```


## Backward compatibility

- Compatibility with LAWICEL CAN ASCII protocol is mandatory.
- Compatibility with the SLCAN protocol, including older versions in this repository, is not required.
- Nevertheless, breaking changes should be limited to major releases.


## Test policy

- Review individual test cases as needed before submitting a pull request.
- Confirm all standard test cases before releasing firmware.
- Visually inspect the pre-release checklist prior to release.

## Test coverage

- Every requirement described in the documents under `doc/` should be covered by at least one test.
- Beyond that, branch or edge-case coverage is added at the developer's discretion — wherever there is doubt about correctness, add a test.
