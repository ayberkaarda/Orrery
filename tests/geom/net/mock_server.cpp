#include "mock_server.h"

#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <csignal>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <fstream>
#include <random>
#include <stdexcept>

namespace orrery::geom::net::test {

namespace {

#ifdef _WIN32
using Socket = SOCKET;
const Socket kNoSocket = INVALID_SOCKET;

void close_socket(Socket s) {
    closesocket(s);
}

struct WinsockSession {
    WinsockSession() {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
    }
    WinsockSession(const WinsockSession&) = delete;
    WinsockSession& operator=(const WinsockSession&) = delete;
    WinsockSession(WinsockSession&&) = delete;
    WinsockSession& operator=(WinsockSession&&) = delete;
    ~WinsockSession() { WSACleanup(); }
};

void ensure_sockets() {
    static const WinsockSession session;
}

bool wait_readable(Socket s, int timeout_ms) {
    WSAPOLLFD fd{};
    fd.fd = s;
    fd.events = POLLRDNORM;
    return WSAPoll(&fd, 1, timeout_ms) > 0;
}

void set_receive_timeout(Socket s, int timeout_ms) {
    const DWORD value = static_cast<DWORD>(timeout_ms);
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&value), sizeof value);
}

long send_bytes(Socket s, const char* data, std::size_t size) {
    return send(s, data, static_cast<int>(size), 0);
}

long receive_bytes(Socket s, char* data, std::size_t size) {
    return recv(s, data, static_cast<int>(size), 0);
}

int ssl_fd(Socket s) {
    return static_cast<int>(s);
}
#else
using Socket = int;
const Socket kNoSocket = -1;

void close_socket(Socket s) {
    close(s);
}

void ensure_sockets() {
    // A client that aborts a transfer must not kill the test with SIGPIPE.
    static const bool ignored = std::signal(SIGPIPE, SIG_IGN) != SIG_ERR;
    (void)ignored;
}

bool wait_readable(Socket s, int timeout_ms) {
    pollfd fd{};
    fd.fd = s;
    fd.events = POLLIN;
    return poll(&fd, 1, timeout_ms) > 0;
}

void set_receive_timeout(Socket s, int timeout_ms) {
    timeval value{};
    value.tv_sec = timeout_ms / 1000;
    value.tv_usec = static_cast<decltype(value.tv_usec)>(timeout_ms % 1000) * 1000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &value, sizeof value);
}

long send_bytes(Socket s, const char* data, std::size_t size) {
    return send(s, data, size, MSG_NOSIGNAL);
}

long receive_bytes(Socket s, char* data, std::size_t size) {
    return recv(s, data, size, 0);
}

int ssl_fd(Socket s) {
    return s;
}
#endif

struct BioDeleter {
    void operator()(BIO* bio) const noexcept { BIO_free(bio); }
};
struct KeyDeleter {
    void operator()(EVP_PKEY* key) const noexcept { EVP_PKEY_free(key); }
};
struct KeyContextDeleter {
    void operator()(EVP_PKEY_CTX* ctx) const noexcept { EVP_PKEY_CTX_free(ctx); }
};
struct CertDeleter {
    void operator()(X509* cert) const noexcept { X509_free(cert); }
};
struct ExtensionDeleter {
    void operator()(X509_EXTENSION* ext) const noexcept { X509_EXTENSION_free(ext); }
};
struct SslDeleter {
    void operator()(SSL* ssl) const noexcept { SSL_free(ssl); }
};
using BioPtr = std::unique_ptr<BIO, BioDeleter>;
using KeyPtr = std::unique_ptr<EVP_PKEY, KeyDeleter>;
using CertPtr = std::unique_ptr<X509, CertDeleter>;

void require(bool ok, const char* what) {
    if (!ok) {
        throw std::runtime_error(std::string{"test certificate: "} + what);
    }
}

void add_extension(X509* cert, int nid, const char* value) {
    X509V3_CTX ctx{};
    X509V3_set_ctx(&ctx, cert, cert, nullptr, nullptr, 0);
    const std::unique_ptr<X509_EXTENSION, ExtensionDeleter> ext{
        X509V3_EXT_conf_nid(nullptr, &ctx, nid, value)};
    require(ext != nullptr && X509_add_ext(cert, ext.get(), -1) == 1, "extension");
}

std::string bio_text(BIO* bio) {
    char* data = nullptr;
    const long size = BIO_ctrl(bio, BIO_CTRL_INFO, 0, &data);
    require(size > 0 && data != nullptr, "PEM output");
    return std::string{data, static_cast<std::size_t>(size)};
}

BioPtr memory_bio(const std::string& text) {
    BioPtr bio{BIO_new_mem_buf(text.data(), static_cast<int>(text.size()))};
    require(bio != nullptr, "memory BIO");
    return bio;
}

const char* reason_phrase(int status) {
    switch (status) {
        case 200:
            return "OK";
        case 301:
            return "Moved Permanently";
        case 302:
            return "Found";
        case 307:
            return "Temporary Redirect";
        case 404:
            return "Not Found";
        default:
            return "Status";
    }
}

} // namespace

TestCertificate make_test_certificate(const std::filesystem::path& dir,
                                      std::string_view subject_alt_names) {
    const std::unique_ptr<EVP_PKEY_CTX, KeyContextDeleter> key_ctx{
        EVP_PKEY_CTX_new_from_name(nullptr, "EC", nullptr)};
    require(key_ctx != nullptr, "key context");
    require(EVP_PKEY_keygen_init(key_ctx.get()) == 1, "keygen init");
    require(EVP_PKEY_CTX_set_group_name(key_ctx.get(), "P-256") == 1, "curve");
    EVP_PKEY* raw_key = nullptr;
    require(EVP_PKEY_generate(key_ctx.get(), &raw_key) == 1, "key generation");
    const KeyPtr key{raw_key};

    const CertPtr cert{X509_new()};
    require(cert != nullptr, "allocation");
    std::random_device device;
    require(X509_set_version(cert.get(), X509_VERSION_3) == 1, "version");
    require(ASN1_INTEGER_set(X509_get_serialNumber(cert.get()),
                             static_cast<long>(device() & 0x3FFFFFFFU) + 1L) == 1,
            "serial");
    require(X509_gmtime_adj(X509_getm_notBefore(cert.get()), -3600) != nullptr, "not before");
    require(X509_gmtime_adj(X509_getm_notAfter(cert.get()), 86400) != nullptr, "not after");
    require(X509_set_pubkey(cert.get(), key.get()) == 1, "public key");
    X509_NAME* name = X509_get_subject_name(cert.get());
    const std::string common_name = "Orrery test server";
    require(X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                                       reinterpret_cast<const unsigned char*>(common_name.c_str()),
                                       -1, -1, 0) == 1,
            "subject");
    require(X509_set_issuer_name(cert.get(), name) == 1, "issuer");
    add_extension(cert.get(), NID_basic_constraints, "critical,CA:TRUE");
    add_extension(cert.get(), NID_key_usage, "critical,keyCertSign,digitalSignature");
    add_extension(cert.get(), NID_subject_alt_name, std::string{subject_alt_names}.c_str());
    add_extension(cert.get(), NID_subject_key_identifier, "hash");
    require(X509_sign(cert.get(), key.get(), EVP_sha256()) > 0, "signature");

    TestCertificate out;
    {
        const BioPtr bio{BIO_new(BIO_s_mem())};
        require(bio != nullptr && PEM_write_bio_X509(bio.get(), cert.get()) == 1, "cert PEM");
        out.certificate_pem = bio_text(bio.get());
    }
    {
        const BioPtr bio{BIO_new(BIO_s_mem())};
        require(bio != nullptr && PEM_write_bio_PrivateKey(bio.get(), key.get(), nullptr, nullptr,
                                                           0, nullptr, nullptr) == 1,
                "key PEM");
        out.private_key_pem = bio_text(bio.get());
    }
    out.ca_file = dir / "test-ca.pem";
    std::ofstream file(out.ca_file, std::ios::binary | std::ios::trunc);
    file << out.certificate_pem;
    require(static_cast<bool>(file), "CA file");
    return out;
}

struct MockServer::Impl {
    Mode mode = Mode::tls;
    SSL_CTX* tls = nullptr;
    Socket listener = kNoSocket;
    int port = 0;
    std::atomic<bool> stop{false};
    std::thread thread;
    mutable std::mutex mutex;
    std::map<std::string, MockResponse> routes;
    std::map<std::string, int> counts;
    int total = 0;

    Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;
    ~Impl() {
        if (listener != kNoSocket) {
            close_socket(listener);
        }
        if (tls != nullptr) {
            SSL_CTX_free(tls);
        }
    }

    // Writes everything or gives up (the client may have gone away).
    bool write_all(Socket client, SSL* ssl, std::string_view data) const {
        while (!data.empty()) {
            const std::size_t chunk = std::min<std::size_t>(data.size(), std::size_t{16} * 1024U);
            long written = 0;
            if (ssl != nullptr) {
                written = SSL_write(ssl, data.data(), static_cast<int>(chunk));
            } else {
                written = send_bytes(client, data.data(), chunk);
            }
            if (written <= 0) {
                return false;
            }
            data.remove_prefix(static_cast<std::size_t>(written));
        }
        return true;
    }

    std::string read_request(Socket client, SSL* ssl) const {
        std::string request;
        char buffer[2048];
        while (request.find("\r\n\r\n") == std::string::npos &&
               request.size() < std::size_t{16} * 1024U) {
            long got = 0;
            if (ssl != nullptr) {
                got = SSL_read(ssl, buffer, static_cast<int>(sizeof buffer));
            } else {
                got = receive_bytes(client, buffer, sizeof buffer);
            }
            if (got <= 0) {
                break;
            }
            request.append(buffer, static_cast<std::size_t>(got));
        }
        return request;
    }

    void pause(std::chrono::milliseconds duration) const {
        const auto until = std::chrono::steady_clock::now() + duration;
        while (!stop.load() && std::chrono::steady_clock::now() < until) {
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
    }

    void respond(Socket client, SSL* ssl) {
        const std::string request = read_request(client, ssl);
        const auto first_space = request.find(' ');
        const auto second_space = request.find(' ', first_space + 1);
        if (first_space == std::string::npos || second_space == std::string::npos) {
            return;
        }
        const std::string path = request.substr(first_space + 1, second_space - first_space - 1);
        MockResponse response;
        {
            const std::lock_guard lock{mutex};
            ++counts[path];
            ++total;
            const auto it = routes.find(path);
            if (it == routes.end()) {
                response.status = 404;
                response.body = "not found";
            } else {
                response = it->second;
            }
        }
        std::string head = "HTTP/1.1 " + std::to_string(response.status) + " " +
                           reason_phrase(response.status) + "\r\n";
        for (const auto& [key, value] : response.headers) {
            head += key;
            head += ": ";
            head += value;
            head += "\r\n";
        }
        if (response.send_content_length) {
            head += "Content-Length: " +
                    std::to_string(response.declared_length.value_or(response.body.size())) +
                    "\r\n";
        }
        head += "Connection: close\r\n\r\n";
        if (!write_all(client, ssl, head)) {
            return;
        }
        std::string_view body = response.body;
        if (response.trickle_interval) {
            while (!body.empty() && !stop.load()) {
                if (!write_all(client, ssl, body.substr(0, 1))) {
                    return;
                }
                body.remove_prefix(1);
                pause(*response.trickle_interval);
            }
            return;
        }
        if (response.stall_after_bytes) {
            const std::size_t sent = std::min(*response.stall_after_bytes, body.size());
            if (!write_all(client, ssl, body.substr(0, sent))) {
                return;
            }
            pause(response.stall_for);
            return;
        }
        (void)write_all(client, ssl, body);
    }

    void serve(Socket client) {
        set_receive_timeout(client, 5000);
        if (mode == Mode::plain) {
            respond(client, nullptr);
            close_socket(client);
            return;
        }
        const std::unique_ptr<SSL, SslDeleter> ssl{SSL_new(tls)};
        if (ssl != nullptr && SSL_set_fd(ssl.get(), ssl_fd(client)) == 1 &&
            SSL_accept(ssl.get()) == 1) {
            respond(client, ssl.get());
            SSL_shutdown(ssl.get());
        }
        close_socket(client);
    }

    void run() {
        while (!stop.load()) {
            if (!wait_readable(listener, 50)) {
                continue;
            }
            const Socket client = accept(listener, nullptr, nullptr);
            if (client != kNoSocket) {
                serve(client);
            }
        }
    }
};

MockServer::MockServer(Mode mode, const TestCertificate* certificate)
    : impl_(std::make_unique<Impl>()) {
    ensure_sockets();
    impl_->mode = mode;
    if (mode == Mode::tls) {
        require(certificate != nullptr, "a TLS server needs a certificate");
        impl_->tls = SSL_CTX_new(TLS_server_method());
        require(impl_->tls != nullptr, "server context");
        const BioPtr cert_bio = memory_bio(certificate->certificate_pem);
        const CertPtr cert{PEM_read_bio_X509(cert_bio.get(), nullptr, nullptr, nullptr)};
        const BioPtr key_bio = memory_bio(certificate->private_key_pem);
        const KeyPtr key{PEM_read_bio_PrivateKey(key_bio.get(), nullptr, nullptr, nullptr)};
        require(cert != nullptr && key != nullptr, "PEM parse");
        require(SSL_CTX_use_certificate(impl_->tls, cert.get()) == 1 &&
                    SSL_CTX_use_PrivateKey(impl_->tls, key.get()) == 1,
                "server certificate");
    }

    impl_->listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    require(impl_->listener != kNoSocket, "socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = 0;
    require(inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) == 1, "loopback address");
    require(bind(impl_->listener, reinterpret_cast<const sockaddr*>(&address), sizeof address) == 0,
            "bind");
    require(listen(impl_->listener, 16) == 0, "listen");
    sockaddr_in bound{};
    socklen_t length = sizeof bound;
    require(getsockname(impl_->listener, reinterpret_cast<sockaddr*>(&bound), &length) == 0,
            "getsockname");
    impl_->port = ntohs(bound.sin_port);
    impl_->thread = std::thread([impl = impl_.get()] { impl->run(); });
}

MockServer::~MockServer() {
    impl_->stop.store(true);
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

void MockServer::route(const std::string& path, MockResponse response) {
    const std::lock_guard lock{impl_->mutex};
    impl_->routes[path] = std::move(response);
}

std::string MockServer::url(std::string_view path) const {
    return std::string{impl_->mode == Mode::tls ? "https" : "http"} +
           "://127.0.0.1:" + std::to_string(impl_->port) + std::string{path};
}

int MockServer::requests(const std::string& path) const {
    const std::lock_guard lock{impl_->mutex};
    const auto it = impl_->counts.find(path);
    return it == impl_->counts.end() ? 0 : it->second;
}

int MockServer::total_requests() const {
    const std::lock_guard lock{impl_->mutex};
    return impl_->total;
}

} // namespace orrery::geom::net::test
