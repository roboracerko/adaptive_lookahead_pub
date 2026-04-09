# Anonymization Notes

This repository is prepared as an anonymized review package.

## Included Safeguards

- top-level documentation avoids submission title, author, and affiliation fields
- package metadata uses anonymous maintainer information
- benchmark scripts rewrite packaged configs to the active workspace path
- `tools/review_audit.sh` scans for common author-identifying strings and local paths

## Still Required Before Public Upload

- rewrite existing git commit history if author metadata is already present
- verify the remote URL and release tag names are anonymized
- replace `CITATION.md` with the final camera-ready metadata only after review
