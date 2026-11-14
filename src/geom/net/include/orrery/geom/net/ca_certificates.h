#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"

#include <filesystem>
#include <span>
#include <string>
#include <string_view>

// Where the URL loader finds the CA certificates that verify https servers
// outside Windows (docs/url-import-security.md, "CA certificates"). The Linux
// release links OpenSSL and libcurl statically, so their compiled-in default
// locations are those of the machine that built them; the loader therefore
// looks for the certificates of the machine it runs on whenever a download
// starts. Windows uses the system certificate store (CURLSSLOPT_NATIVE_CA).
namespace orrery::geom::net {

// The environment variables that name certificates explicitly; empty means
// unset.
struct CaEnvironment {
    std::string curl_ca_bundle; // CURL_CA_BUNDLE: a PEM bundle (curl's variable)
    std::string ssl_cert_file;  // SSL_CERT_FILE: a PEM bundle (OpenSSL's variable)
    std::string ssl_cert_dir;   // SSL_CERT_DIR: hashed certificate directories, separated
                                // as OpenSSL does (':', ';' on Windows)
};

struct CaCertificates {
    std::filesystem::path bundle; // a PEM bundle (CURLOPT_CAINFO), empty for none
    std::string directories;      // hashed directories (CURLOPT_CAPATH), empty for none
    std::string origin;           // what selected them, for messages
};

// The PEM bundles of the common distributions, in the order they are tried:
// Debian, Ubuntu, Arch, Gentoo; Fedora and RHEL; openSUSE; OpenELEC; RHEL 7
// and CentOS; Alpine and the BSDs.
inline constexpr std::string_view kDefaultCaBundles[] = {
    "/etc/ssl/certs/ca-certificates.crt",
    "/etc/pki/tls/certs/ca-bundle.crt",
    "/etc/ssl/ca-bundle.pem",
    "/etc/pki/tls/cacert.pem",
    "/etc/pki/ca-trust/extracted/pem/tls-ca-bundle.pem",
    "/etc/ssl/cert.pem",
};

// Directories of hashed certificates (<8 hex digits>.<n>), tried after the
// bundles.
inline constexpr std::string_view kDefaultCaDirectories[] = {
    "/etc/ssl/certs",
    "/etc/pki/tls/certs",
};

// CURL_CA_BUNDLE, SSL_CERT_FILE and SSL_CERT_DIR of this process. Always
// empty on Windows, where they are not used.
[[nodiscard]] CaEnvironment ca_environment_from_process();

// Selects the certificates:
//   1. if any variable of `environment` is set, only the variables count:
//      CURL_CA_BUNDLE, or else SSL_CERT_FILE, is the bundle and SSL_CERT_DIR
//      the directories. A variable that names no usable file (or, for
//      SSL_CERT_DIR, no directory at all) is an error, since the user asked
//      for it;
//   2. otherwise the first of `bundles` that is a non-empty file;
//   3. otherwise the first of `directories` that holds a hashed certificate.
// Fails with tls_failure, naming everything that was tried. Verification is
// never turned off for want of certificates.
[[nodiscard]] core::Result<CaCertificates, FetchError>
find_ca_certificates(const CaEnvironment& environment, std::span<const std::string_view> bundles,
                     std::span<const std::string_view> directories);

} // namespace orrery::geom::net
