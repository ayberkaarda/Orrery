# URL model import: security

Orrery can load a model from a URL (spec section 4.3, ADR-010). The download
is untrusted input from an unknown server. This page describes what the
loader (`orrery_geom_net`, `src/geom/net`) defends against, the limits it
enforces, and how the tests prove it.

The loader only downloads, checks and stores files. It does not parse meshes:
the file it returns is passed to the importer (assimp, restricted
post-processing) and then to geometry validation (vertex count, NaN, index
range) before anything reaches the GPU.

## Threat model

The attacker controls the server behind the URL, and anything a redirect
points to. They may:

| Threat | Example |
|---|---|
| Downgrade to cleartext | `http://` URL, or an `https` URL that redirects to `http://`, `ftp://`, `file://` |
| Impersonate the server | self-signed or wrong-host certificate |
| Exhaust disk or memory | 50 GB body, no `Content-Length`, a `Content-Length` that lies |
| Hang the application | headers then silence, or a trickle of bytes |
| Disguise content | an executable or HTML page named `model.stl`, or `Content-Type: model/stl` on other data |
| Decompression bomb | zip entry with a ratio of 1000:1, sizes in the zip header that lie, many entries |
| Write outside the cache | zip entries named `../../x`, `/etc/x`, `C:\x`, symlinks |
| Poison the cache | a modified file in the cache directory |

Not in scope: the parsers themselves (that is the importer's job, behind its
own validation), local attackers with write access to the user's home
directory beyond the cache checks below, and requests to private network
addresses. The URL is typed by the user of a desktop application, so
server-side request forgery does not apply; the tests rely on 127.0.0.1.

## What is checked

**URL policy** (`check_https_url`), applied to the URL and to every redirect
target before it is contacted:

- the scheme must be `https` (case-insensitive); everything else fails with
  `scheme_not_allowed`, before any network or file system access;
- the URL needs a host, may not carry credentials (`user@host`), whitespace,
  control characters or a backslash in the authority, and is at most 2048
  characters long.

**Transport** (libcurl over OpenSSL):

- libcurl itself is restricted to `https` (`CURLOPT_PROTOCOLS_STR`, and
  `CURLOPT_REDIR_PROTOCOLS_STR` as a second layer);
- redirects are not followed by libcurl. The loader reads each `Location`,
  checks it with the URL policy (`redirect_not_allowed` otherwise) and counts
  hops (`too_many_redirects` beyond the limit). An `https` to `http` redirect
  is therefore refused before the `http` server is contacted;
- peer and host name verification are always on, TLS 1.2 or newer. On
  Windows the system certificate store is used (`CURLSSLOPT_NATIVE_CA`);
  elsewhere OpenSSL's system bundle;
- only a final `200` is accepted; redirect bodies are discarded (capped at
  64 KiB);
- the `Content-Type` is checked as soon as the headers arrive (see below);
  a rejected type aborts the transfer before the body is stored;
- no transparent decompression (`Content-Encoding` is not decoded), so the
  bytes counted are the bytes stored;
- environment proxies are honoured unless `use_environment_proxy` is false.

**Size**: a declared `Content-Length` above the cap is refused before the body
(`CURLOPT_MAXFILESIZE_LARGE`); independently, the write callback counts every
body byte and aborts the moment the cap is exceeded, which covers chunked
bodies, bodies without a length, and servers that send more than they
declared.

**Time**: connect timeout, a total timeout over all hops, and a low-speed
guard (abort when fewer than N bytes per second arrive for a window). These
limits bound a fetch nobody stops; a user who does not want to wait cancels
it (see [Cancellation](#cancellation)).

**Content**: the format is decided by magic bytes and structure only. The
URL path, file name and extension are never consulted:

| Format | Evidence |
|---|---|
| zip | `PK\x03\x04` |
| GLB | `glTF`, version 2, header length equal to the file size |
| FBX | `Kaydara FBX Binary  \0`, or ASCII starting with `; FBX` |
| STL binary | 84-byte header with a triangle count n > 0 and size = 84 + 50 n |
| PLY | `ply` line, a `format ascii`, `binary_little_endian` or `binary_big_endian` line, then an `end_header` line |
| STL ASCII | text starting with `solid` and containing `facet` or `endsolid` |
| glTF | text starting with `{` and containing `"asset"` |
| OBJ | text whose every line is empty, a comment or an OBJ statement, with at least one `v` |

Text formats must be text (no NUL, no control bytes except whitespace).
For PLY only the header up to and including the `end_header` line has to be
text: a binary PLY body holds raw coordinates and indices, where NUL and
control bytes are normal. The body of an ASCII PLY must be text as well.
Anything else fails with `unrecognised_content`: executables, HTML, scripts.

**MIME**: the `Content-Type` must be absent, generic
(`application/octet-stream`, `binary/octet-stream`, `text/plain`) or a known
model or zip type. A specific type must agree with the detected content
(`model/stl` on an OBJ file is `mime_rejected`). `text/html`, executables,
scripts and images are refused.

**Zip** (single level, miniz): the whole central directory is validated
before anything is extracted:

- at most 1000 entries;
- every name is a plain relative path: no leading `/` or `\`, no `:` (drive
  letters, alternate streams), no `..` component with either separator, no
  control characters;
- no symbolic links (Unix mode in the external attributes), no encrypted
  entries, only stored or deflated data;
- for each entry, uncompressed size / compressed size must not exceed 100;
  the sum of uncompressed sizes must not exceed 200 MB;
- during extraction the real output is counted, and an entry that produces
  more bytes than its header declared is rejected as a bomb (a lying header
  cannot bypass the ratio check);
- only entries whose content is a supported model are written. The first
  bytes are held in memory until the format is known; other entries (text,
  images, executables, nested archives) never reach the disk;
- archive names are never used as file system paths. Extracted files are
  named `NNNN-<stem><ext>` (index, the name reduced to `[A-Za-z0-9_-]`, and
  the extension of the detected format);
- every extracted file is *created*, never opened: exclusive creation
  (`O_EXCL` on POSIX, `CREATE_NEW` with `FILE_FLAG_OPEN_REPARSE_POINT` on
  Windows) fails if anything already has that name, so a hard link, a
  symbolic link (also a dangling one) or a junction planted in the output
  directory makes the extraction fail (`io_failure`) instead of being
  written through. `CREATE_NEW` alone is not enough on Windows: it follows a
  dangling symbolic link and creates its target.

The first model in archive order is returned; its name in the archive is
`FetchedModel::archive_member`. Companion files (`.mtl`, external glTF
buffers, textures) are not extracted.

## Limits

| Limit | Default | Option |
|---|---|---|
| Download size | 200 MB (209 715 200 bytes) | `FetchLimits::max_download_bytes` |
| Redirect hops | 5 | `FetchLimits::max_redirects` |
| Connect timeout | 15 s | `FetchLimits::connect_timeout` |
| Total time | 300 s | `FetchLimits::total_timeout` |
| Low-speed abort | < 1024 B/s for 30 s | `low_speed_bytes_per_second`, `low_speed_window` |
| Zip entries | 1000 | `ArchiveLimits::max_entries` |
| Zip compression ratio | 100 | `ArchiveLimits::max_compression_ratio` |
| Zip total uncompressed | 200 MB | `ArchiveLimits::max_total_uncompressed_bytes` |
| URL length | 2048 | `kMaxUrlLength` |

## Cache

Downloads are content-addressed by the SHA-256 of the downloaded bytes:

```
<cache>/<sha256>/source<ext>      the download (extension from the content)
<cache>/<sha256>/NNNN-<stem><ext> models extracted from a zip
<cache>/<sha256>/entry.txt        source and model file names, model format, archive member
<cache>/urls/<sha256 of the URL>  content address last served by that URL
<cache>/.staging/<random>/        temporary directory of a download in progress
```

`<cache>` is `$XDG_CACHE_HOME/orrery/models` or `~/.cache/orrery/models` on
Linux, `~/Library/Caches/orrery/models` on macOS and
`%LOCALAPPDATA%\orrery\cache\models` on Windows (`FetchOptions::cache_dir`
overrides it).

A download goes to a new staging directory inside the cache (the temporary
directory, on the same volume so that the final step is a rename). The
directory is renamed to its content address only when it is complete, and it
is removed on any failure, so a rejected download leaves nothing behind.

Nothing stored in the cache is trusted; the content address is the only
binding. A URL that was fetched before is served from the cache without a
request only if all of this holds again:

- the source file hashes to the name of its directory. A digest recorded in
  `entry.txt` would prove nothing, because whoever can replace a file can
  rewrite `entry.txt` too;
- the model is derived from that verified source again: for a plain download
  the model *is* the source file; for a zip the archive is extracted afresh
  into a staging directory and the cached model must be byte-identical to the
  first model of that extraction (same file name, format and member);
- the source passes **this request's** policy as a fresh download would: its
  size is within `max_download_bytes`, its sniffed format matches, and a zip
  passes the current `ArchiveLimits` (entry count, total size, ratio). A
  model cached under permissive limits is not served under stricter ones.

If any check fails the entry is ignored and the URL is downloaded again, so
the result (the model, or the same policy error) is the one an uncached fetch
gives. When a download is committed and an entry already exists under its
address, the existing entry is kept only if its source and model are
byte-identical to what was just staged; otherwise the fresh download replaces
it, also with `use_cache = false`.

Links are never followed. `urls/`, `.staging/` and every entry directory must
be real directories: a symbolic link or a Windows junction there makes
`prepare()` fail with `cache_failure` (for `urls/` and `.staging/`) or makes
the entry a miss that is replaced (for an entry directory; the link itself is
removed, never the directory it points to). Source and model files must be
regular files, not links. The cache file names read back from `entry.txt`
must be plain file names, and the URL index must hold a 64-digit hex digest,
so a modified index cannot point outside the cache.

What the cache does not claim, by design:

- **The cache root itself may be a link.** A user may legitimately relocate
  `~/.cache` or `%LOCALAPPDATA%` with a link, and whoever controls the root
  path controls the whole cache anyway. Only the directories the loader
  creates inside it are checked.
- **The URL index is a hint.** Someone who can write the cache can point
  `urls/<hash of a URL>` at another valid entry. That entry still passes every
  check above (address, re-derived model, current limits), so the worst case
  is a well-formed model from another download, never unchecked bytes; the
  model is still untrusted input for the importer. Defending the mapping
  itself would need a key the same local user could not read, which is out
  of scope (see the threat model).
- **The `Content-Type` is not re-checked on a hit.** It is a property of a
  past HTTP response, not of the stored bytes; it was checked when the entry
  was downloaded, and the content sniffing that it is compared against is
  repeated on every hit.
- **A hit that fails the current policy costs a request.** Instead of
  failing from the cache, the loader downloads again so that the outcome is
  exactly that of an uncached fetch; offline, the error is then a network
  error rather than the policy error.

## Cancellation

`FetchOptions::cancel_requested` is a callback the loader polls while a fetch
runs: before the first request and before every redirect hop, on every block
of the body (libcurl's write callback), and from libcurl's progress callback
(`CURLOPT_XFERINFOFUNCTION`), which libcurl calls frequently while data
arrives and about once a second while a server sends nothing. Returning true
ends the fetch with `FetchErrorKind::cancelled` within about a second, also
when the server stalls after its headers or trickles one byte at a time,
cases the limits above would only end after 30 s (low-speed guard) or 300 s
(total). The partial download lives only in the fetch's staging directory,
which is removed; nothing is committed to the cache and the URL index is not
touched. Without a callback (the default) the transfer is configured exactly
as before. Every limit stays in force with or without one.

The Aero workspace wires the stop token of its background task into the
callback (`sim::fetch_model_url`), so its Cancel button and closing the
window end a stalled download within about a second
(aero-workspace.md, "Approximations and limits").

## Experiments

An aero experiment that uses a URL model records the `url` and the SHA-256
of the download. When the cached file named in the experiment is gone,
opening it would mean contacting the server again, so the application never
does that on its own:

- **Permission first.** The window shows the prompt "This experiment wants
  to download a model from `<host>`" with the URL and the recorded SHA-256,
  and downloads only after **Download**; **Cancel** leaves the experiment
  without geometry. A headless run downloads only with `--allow-download`
  (app.md, "Command line"); without it the run fails before any network
  access with an error that names the host, URL and recorded digest.
  `sim::load_experiment_geometry` refuses by default
  (`sim::ExperimentDownload::allowed`).
- **Digest before parsing.** The recorded digest is passed to the loader as
  `FetchOptions::expected_sha256`. The loader compares it with the SHA-256
  it computes while streaming, as soon as the download is complete and
  before the content is sniffed, extracted or handed to the importer. A
  mismatch fails with `FetchErrorKind::digest_mismatch` (shown as "The model
  changed at its source", with both digests) and the staging directory is
  removed, so the bytes are never parsed and never cached. A cached entry
  under another digest is not served for such a request (nor re-extracted);
  the URL is downloaded and checked again. For a zip the digest is that of
  the archive, as recorded.
- An experiment without a recorded digest is downloaded after the same
  permission, and the prompt says that none was recorded.

## Licence and source

`FetchOptions::license_note` is free text entered by the user (licence,
author, source). It is carried unchanged into `FetchedModel::license_note`,
together with `source_url` and the SHA-256, so the UI can show where a model
came from. The program does not check licences; responsibility stays with the
user (ADR-010).

## Design decisions

These refine ADR-010 without changing it:

- **One TLS stack everywhere.** libcurl is built with OpenSSL on Windows and
  Linux (vcpkg `curl[openssl]`, no other protocols or features). On Windows
  the system store is used through `CURLSSLOPT_NATIVE_CA`. One backend means
  one set of TLS behaviours to test, and the CA override below works the same
  on every platform.
- **Redirects are followed by the loader, not by libcurl**, so the https rule
  is enforced by Orrery's own URL policy on every hop (and again by libcurl's
  protocol restriction).
- **A separate target.** `orrery_geom_net` depends only on `orrery_core`,
  libcurl and miniz. Mesh import stays in `orrery_geom`; code that never
  downloads does not link a network stack.
- **SHA-256 is implemented in the project** (tested against the FIPS 180-4
  vectors) so the library needs no direct crypto dependency for hashing.

## How the tests prove it

`tests/geom/net` (ctest names `net.*`) runs offline and deterministically. The
transport tests use an in-process mock server on 127.0.0.1 with an ephemeral
port. For TLS it uses a self-signed certificate (P-256, `IP:127.0.0.1`)
generated when the test runs, so no key material is checked in.

The certificate reaches the loader through `FetchOptions::ca_bundle_override`,
which *replaces* the system trust store with that PEM file. It does not
disable verification: peer and host name checks stay on, and `https` is still
the only protocol. The default is empty and production code never sets it.
This keeps the real libcurl configuration under test: the https-only rule, the
redirect checks, the size and time limits all run against a real TLS server,
not a mock of libcurl. The same server, with the override cleared or pointing
at another CA, proves that an untrusted certificate is refused. Those cases
fail in the chain check, so they would still pass if host name verification
were switched off; a separate case trusts a certificate whose only name is
`DNS:other-host.invalid` and requests `https://127.0.0.1`, which only host
name verification (`CURLOPT_SSL_VERIFYHOST = 2`) can refuse.

Negative cases covered: `http`, `HTTP`, `ftp` and `file` URLs (and no request
reaches a plain HTTP server); `https` to `http` and to `ftp` redirects (the
`http` target is never contacted); a redirect loop; untrusted certificates; a trusted certificate for another
host name;
a declared size over the cap, a 50 GiB `Content-Length`, an undeclared
streamed body over the cap and an understated `Content-Length`; an executable
served as `.stl`, HTML served as `text/html`, a valid model with an executable
MIME type and a mislabelled model type; a stalled server (total timeout) and a
trickling server (low-speed guard); a cancelled fetch, with a server that
sends one byte every 200 ms, with a server silent after ten bytes (both end
within two seconds of the cancellation, leaving neither a cache entry, a URL
index entry nor a staging directory behind; a normal fetch with a callback
that never cancels is unchanged) and before the first request (no request is
sent); a download whose digest is not the expected one (discarded unread,
also when an entry under another digest is cached); a non-200 status; zip bombs (ratio about
1000, a lying size header, a total size over the cap), path traversal and
absolute names, symlinks, encrypted entries, too many entries, archives
without a model or with only a nested archive, corrupt archives; a hard link,
a symbolic link and a dangling symbolic link planted at an extraction name
(the link target stays untouched, and a dangling target is not created); a
tampered cache entry; a replaced model (plain and zip) whose recorded digest
was rewritten to match, on a hit and on a forced refresh; a cached model under
a tightened download size, entry count, total size and compression ratio
limit; `urls/` and `.staging/` replaced by a link (junction on Windows), and
an entry directory replaced by one. Positive cases: an STL download with its
hash, licence note and cache path; a cache hit that does not download again;
https redirects; a zip download that yields its first model and is served
from the cache; little- and big-endian binary PLY with zero coordinates,
detected, downloaded and extracted from a zip.
