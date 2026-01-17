# Contributing

Feel free to open an issue, start a discussion, fork this repository,
or give a star to your favorite repository.
These actions will help me discover better software.
Pull requests are not expected to be accepted here.


## Repository structure

```
/
└── doc                     # Protocol and firmware documentation
│   └── usb2canfdv1.wiki    # Git subtree (external wiki repo)
├── test                    # Test scripts for system test
│   └── slcan-tester        # Git subtree (external repo)
└── usb2canfdv1-fw          # Firmware source code
```

The commands below sync changes between this repository and subtrees
([wiki](https://github.com/Nakakiyo092/usb2canfdv1/wiki) and [slcan-tester](https://github.com/Nakakiyo092/slcan-tester)).  
`gh auth login` can be used to login to github.com.

```
git subtree pull --prefix=doc doctree main --squash
git subtree push --prefix=doc doctree main
git subtree pull --prefix=test testtree main --squash
git subtree push --prefix=test testtree main
```

## Backward compatibility

TODO


## Test policy

- Review individual test cases as needed before submitting a pull request.
- Confirm all standard test cases before releasing firmware.
- Visually inspect the pre-release checklist prior to release.
