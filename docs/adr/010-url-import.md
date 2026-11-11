# ADR-010: Restricted URL model import

## Context
Loading models from URLs exposes the application to oversized downloads, spoofed file types, decompression bombs and malformed meshes that parsers may mishandle.

## Decision
https only; 200 MB size cap; MIME and magic-byte checks instead of trusting the extension; download to a temporary directory, hash with SHA-256 and cache by hash; single-level zip with a compression ratio limit of 100; restricted assimp post-processing; geometry validated (vertex count, NaN, index range) before any GPU upload.

## Consequences
Reduced attack surface and a reproducible cache. Some legitimate large files or nested archives are refused. Licensing is the responsibility of the user; a source note field is provided.

Alternatives rejected: unrestricted download.

## Status
Accepted
