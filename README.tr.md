# Orrery

Orrery, OpenGL 4.6 arayüzlü, C++20 ile yazılmış bir masaüstü fizik
laboratuvarıdır. İki alanı tek uygulamada bir araya getirir ve birbirine
bağlar:

- **Uzay dinamiği**: simplektik ve uyarlamalı integratörlerle N-cisim
  kütleçekimi, kara delikler (sözde-Newton dinamiği, render tarafında
  Schwarzschild ışık bükülmesi), çarpışmalar ve gelgit parçalanması.
- **Aerodinamik**: seçilen ya da içe aktarılan bir 3B cisim etrafında akış;
  üç rejim için üç çözücü - lattice Boltzmann, panel yöntemi ve değiştirilmiş
  Newton teorisi - kuvvet katsayıları, basınç haritaları ve akış alanları
  üretir.
- **Atmosferik giriş**: atmosfer modelleri ve aerodinamik çözücüden
  örneklenen bir sürüklenme katsayısı tablosu, bir sondanın atmosfere
  girişini sürer.

Amaç bir oyun değil; gözlemlenebilir, doğrulanabilir ve deterministik bir
araçtır. Her fiziksel basitleştirme kodda işaretlenir ve
[docs/physics.md](docs/physics.md) içinde listelenir; hiçbir çözücü
doğrulanmış CFD iddiasında bulunmaz.

Sürüm 0.0.1. Windows (MSVC 2022) ve Linux (GCC 13, Clang 17) desteklenir;
macOS kapsam dışıdır (OpenGL 4.6 yok).

## İçindekiler

- [Özellikler](#özellikler)
- [Derleme](#derleme)
- [Çalıştırma](#çalıştırma)
- [Yapılandırma](#yapılandırma)
- [Doğrulama](#doğrulama)
- [Sınırlamalar](#sınırlamalar)
- [Depo düzeni](#depo-düzeni)
- [Belgeler](#belgeler)

## Özellikler

**Uygulama.** Üç çalışma alanı - Space, Aero ve Entry - her birinin kendi
ana çubuğu, yerleştirilmiş panelleri ve kamerası vardır. Fizik kendi
iş parçacıklarında çalışır ve render tarafına değişmez anlık görüntülerle
ulaşır; hiçbir kare bir çözücüyü beklemez. Her çalıştırma bir deney olarak
kaydedilebilir (JSON: girdiler, koşullar, çözücü, sonuç), listelenebilir,
karşılaştırılabilir ve yeniden açılabilir.

**Space çalışma alanı.** Hazır deneyler: J2000 Güneş sistemi,
Güneş-Dünya-Ay, üç cisimli sekiz figürü, Sgr A* ve S2 yıldızı, tek başına bir
kara delik.

- İntegratörler: leapfrog (kick-drift-kick, varsayılan), 4. mertebe Yoshida,
  IAS15 (uyarlamalı), RK4 (karşılaştırma için); doğrudan toplam ya da
  Barnes-Hut sekizli ağacı; kara deliklerin yakınında Paczynski-Wiita
  potansiyeli.
- Tamsayı zaman tutan sabit adımlı biriktirici: N adımdan sonraki durum, kare
  hızı ne olursa olsun bit düzeyinde aynıdır.
- 1x ile 1e7x arasında zaman hızlandırma. Bir **zaman hızlandırma bütçesi**
  fiziği duvar saati zamanının bir payı içinde tutar: istenen hız sığmadığında
  hız, sığan en yüksek düzeye indirilir; bu, `LIMITED` etiketiyle ve System
  Monitor'da istenen ile ulaşılan hız ve adım maliyetiyle gösterilir; yük
  düştüğünde hız geri gelir (histerezisle, titreşmez).
- Paneller: Body Inspector (seçili cismin oskülatör yörüngesi), System
  Monitor (enerji kayması, adım süresi, zaman hızlandırma), Experiment (uzay
  deneylerini kaydetme ve açma), Render Settings.
- Render: kayan orijin, logaritmik derinlik, izler, yıldız alanı, ACES ton
  eşlemeli HDR ve analitik yıldız parlaması (bloom geçişi yok); ışın yürütmeli Schwarzschild merceklenme geçişi ve Doppler
  parlaması ile kütleçekimsel kırmızıya kaymalı ince birikim diskiyle kara
  delikler.

**Aero çalışma alanı.**

- Geometri: parametrik primitifler (küre, küp, silindir, koni, elipsoit, düz
  plaka, kama, Apollo ve Orion tipi kapsüller, NACA 4 haneli kesitler ve
  kanatlar), diskten ya da bir https URL'sinden OBJ, STL, glTF/GLB, PLY ve FBX
  içe aktarma (boyut, tür ve zip sınırları, SHA-256 önbelleği), birim ve
  yönelim onayı, raporlu otomatik onarım. İçe aktarılan modeller
  "unvalidated geometry" rozeti taşır.
- Türetilmiş Reynolds ve Mach sayılarıyla akış koşulları; rejim seçici bir
  çözücü önerir ve geçerlilik aralığı dışında, çalıştırmayı engellemeden
  uyarır.
- Çözücüler: 2B ve 3B panel yöntemi, değiştirilmiş Newton teorisi
  (Sutton-Graves durma noktası ısı akısı tahminiyle), Smagorinsky alt ızgara
  modelli D2Q9 ve D3Q19 lattice Boltzmann (çalışma alanında CPU üzerinde;
  çözücü kütüphanesi için bir GPU hesaplama arka ucu vardır).
- Sonuçlar: ayrıntılı dökümle Cd, Cl, Cm ve yakınsama grafikleri; görünüm
  cisim üzerinde Cp, kuvvet okları, kesit düzlemleri, akım çizgileri,
  parçacıklar ve Q-kriteri eş yüzeyini gösterir. Deneyler hücum açısına göre
  karşılaştırılır (kaldırma eğimi) ve CSV olarak dışa aktarılır.

**Entry çalışma alanı.**

- US Standard Atmosphere 1976, ortalama bir Mars profili (NASA Glenn
  uyumu) ve üstel atmosferler; Dünya ve Mars.
- Sabit kütleli, kaldırmasız bir sonda; Cd(Mach) tablosu elle düzenlenir ya
  da Newton çözücüsünden örneklenir; adımın kütleçekim kısmı için IAS15,
  Yoshida 4 ya da leapfrog.
- Zamana karşı irtifa, hız, g yükü, dinamik basınç ve ısı akısını gösteren
  Entry Profile paneli, yer izi haritası ve g yüküne ya da ısı akısına göre
  renklendirilmiş yörüngenin 3B görünümü. Sonuçlar: iniş, sekme (skip-out) ya
  da yörüngede kalma.

**Kullanılabilirlik.**

- Tüm pencere için tek bir klavye kısayolu tablosu (Space çalıştır ya da
  duraklat, `[` ve `]` zaman hızlandırma, Ctrl+1/2/3 çalışma alanları, Ctrl+S
  kaydet, Ctrl+O kayıtlı deneyler, F1 ya da `?` yardım katmanı, Esc);
  yapılandırma dosyasından özelleştirilebilir, çakışmalar algılanır ve
  raporlanır.
- Sessiz hata yok: hatalar oluştukları yerde gösterilir (durum satırı,
  paneller, içe aktarma hataları için ayrıntılı bir modal pencere); ölümcül
  bir hata nedenini standart hataya ve bir çökme günlüğüne yazar.
- Renk görme farklılıklarında okunabilir kalan renk haritaları; Space düzeni
  testlerde 1920x1080, 1366x768 ve 1024x640 boyutlarında çizilir.

## Derleme

Gereksinimler: CMake 4.1 ya da daha yenisi, Ninja, bir C++20 derleyicisi
(MSVC 2022, GCC 13 ya da Clang 17) ve manifest kipinde
[vcpkg](https://github.com/microsoft/vcpkg) (`vcpkg.json` kütüphaneleri
listeler: GLFW, glad, docking destekli Dear ImGui, ImPlot, glm, Eigen,
assimp, OpenSSL ile libcurl, miniz, oneTBB, nlohmann/json, toml++ ve
Catch2).

Linux:

```sh
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

Windows, Visual Studio 2022 için bir Developer PowerShell içinde (x64):

```powershell
$env:VCPKG_ROOT = "C:\path\to\vcpkg"
cmake --preset release
cmake --build --preset release
ctest --preset release
```

| Ön ayar (Windows / Linux) | Amaç |
|---|---|
| `release` / `linux-release` | pencereli, optimize derleme |
| `debug-asan` / `linux-debug-asan` | AddressSanitizer'lı (Linux'ta UBSan da) hata ayıklama derlemesi |
| `ci-headless` / `linux-ci-headless` | CI derlemesi: pencere yok, uyarılar hata sayılır |

`[gl]` etiketli testler bir OpenGL 4.6 bağlamı ister; bağlam yoksa
atlanırlar, `ORRERY_REQUIRE_GL=1` ayarlıysa başarısız olurlar. Sürekli
entegrasyon, Ubuntu'da GCC 13 ve Clang 17 ile, Windows'ta MSVC ile derler ve
test eder; bir AddressSanitizer işi, clang-format, clang-tidy, cppcheck ve
kütüphane bağımlılık sınırlarının denetimini (`tools/check_deps.py`)
çalıştırır.

## Çalıştırma

```sh
orrery_app                                   # pencere, Güneş sistemi
orrery_app --experiment figure_eight         # adıyla hazır bir deney
orrery_app --experiment path/to/run.json     # ya da herhangi bir deney dosyası
orrery_app --headless --experiment black_hole --out bh.png --size 1920x1080
orrery_app --headless --experiment solar_system --frame 600 --out solar.png
orrery_app --headless --experiment aero_naca0012_panel --out naca.png
orrery_app --headless --experiment entry_apollo_like --out entry.png
orrery_app --help | --version | --modules
```

- Pencere, deneyin (space, aero ya da entry) ya da yapılandırmanın çalışma
  alanında açılır. Kısayol listesi için F1'e basın.
- `--headless` ekran dışında tek bir görüntü çizer ve bir PNG yazar: uzay
  deneyleri, deneyin hızlandırmasıyla `--frame` kez 1/60 s ilerletildikten
  sonra; aero deneyleri çözücü çalıştırıldıktan sonra; entry deneyleri giriş
  sonuna kadar çalıştırıldıktan sonra tüm çalışma alanıyla. Hiçbir zaman
  pencere göstermez ama bir OpenGL 4.6 sürücüsü ister; yoksa 3 çıkış koduyla
  sonlanır.
- `--benchmark N` kare sürelerini yazdırır (pencerede ya da headless
  görüntüden sonra).
- Hazır deneyler `assets/experiments` altındadır: `solar_system`,
  `sun_earth_moon`, `figure_eight`, `sgr_a_s2`, `black_hole`,
  `aero_naca0012_panel`, `aero_naca0012_lbm`, `aero_naca2412_wing`,
  `aero_apollo_capsule`, `entry_apollo_like`.
- Çıkış kodları: 0 başarı, 1 hata, 2 geçersiz komut satırı, 3 OpenGL 4.6
  bağlamı yok.

## Yapılandırma

Platformun yapılandırma dizinindeki `config.toml` (Linux
`$XDG_CONFIG_HOME/orrery` ya da `~/.config/orrery`, Windows
`%APPDATA%\orrery\config`); kaydedilen deneyler ve çökme günlüğü veri
dizinine gider (Linux `$XDG_DATA_HOME/orrery` ya da `~/.local/share/orrery`,
Windows `%APPDATA%\orrery\data`).

```toml
[paths]
experiments = "/home/me/orrery-runs"
[window]
width = 1600
height = 900
workspace = "entry"        # "space", "aero" ya da "entry"
[aero]
steps_per_frame = 200
[space]
physics_budget = 0.5       # fiziğin kullanabileceği duvar saati payı
[shortcuts]
save = "F5"
help = ["F1", "?"]
```

Bilinmeyen anahtarlar ve geçersiz değerler satırlarıyla birlikte standart
hataya raporlanır ve yok sayılır; hiçbir şey sessizce atılmaz. Ayrıntılar:
[docs/app.md](docs/app.md#10-configuration).

## Doğrulama

Neyin neye karşı denetlendiği (hepsi test takımında; ölçülen değerler ve
kurulumları [docs/physics.md](docs/physics.md) ile onun bağlantı verdiği
belgelerdedir):

| Alan | Referans | Sonuç |
|---|---|---|
| İki cisim yörüngeleri | analitik Kepler çözümü | Yoshida4 10 yörünge sonra 2.7e-8 AU; Kepler periyodu 2.4e-10'a kadar |
| Güneş sistemi, 1000 yıl, Yoshida4 | enerji korunumu | en büyük abs(dE/E0) 1.4e-10 (hedef 1e-9) |
| Dairesel yörünge, 100 yörünge, leapfrog | analitik yörünge | 3.8e-7 AU (hedef 1e-6 AU) |
| Determinizm | durum özeti (hash) | aynı adım sayısından sonra, her kare temposunda ve zaman hızlandırma bütçesiyle (belli bir duvar saati süresinde atılan adım sayısını değiştirebilir) bit düzeyinde aynı |
| Paczynski-Wiita yörüngeleri ve ISCO | analitik periyot; 3 r_s civarında kararlılık | periyot hatası 9e-12; üstünde kararlı, altında dalış |
| Kara delik merceklenmesi | zayıf alan sapma serisi, güçlü sapma (Bozza 2002), CPU jeodezikleri | üçüncü mertebe terim içinde; Einstein halkası yarıçapı 0.01 px içinde eşleşir |
| Geometri | kapalı biçimli hacimler ve alanlar | 65 denetimde en kötü göreli hata 6.1e-14 |
| Küre vokselleştirme | gerçek küre | 85184 hücrenin 85184'ü uyuşur |
| NACA 0012, 2B panel yöntemi | ince kanat eğimi rad başına 2 pi; kalın kesit (Joukowski) değerleri | eğim 1.098 x 2 pi (yüzde 10 ölçütü içinde); Cl kalın kesit değerlerinden en fazla yüzde 0.7 farklı |
| AR 6 kanat, 3B panel yöntemi | Helmbold taşıyıcı çizgi tahmini; Trefftz düzlemi | CL yüzde 10 içinde; açıklık verimi 0.988 |
| Apollo kapsülü, Mach 20, Newton | yayımlanmış Cd yaklaşık 1.3 | 20 derece trimde Cd 1.367 (yüzde 5 üstünde), L/D -0.30 |
| Silindir, Re 100, LBM 2B | Strouhal 0.16 ile 0.20, Cd 1.2 ile 1.5 | St 0.167, Cd 1.468, CPU ve GPU'da aynı |
| Küre, Re 10 ile 1000, LBM 3B | Schiller-Naumann bağıntısı, yüzde 15 | yüzde 1.3 ile 10 arasında |
| Akışa dik plaka, LBM 3B | Cd yaklaşık 1.28, yüzde 15 | Cd 1.158 |
| CPU'ya karşı GPU lattice Boltzmann | göreli L2 1e-4 altında | akış oturduktan sonra sağlanır (sınırlamalara bakın) |
| US Standard Atmosphere 1976 | yayımlanmış tablo | katman değerleri basılı hassasiyete kadar |
| Apollo benzeri giriş | tepe 6 ile 8 g; enerji dengesi yüzde 1 içinde | 6.78 g; enerji hatası 2.1e-5 |
| Allen-Eggers balistik giriş | analitik tepe yavaşlama | yüzde 0.5 içinde |

## Sınırlamalar

- Hiçbir çözücü doğrulanmış CFD olarak sunulmaz. Panel yöntemi viskozsuzdur
  (yüzey sürtünmesi ayrı bir düz plaka tahminidir), Newton teorisi Mach 5 ve
  üstü içindir, lattice Boltzmann çözücüsü düşük Mach yöntemidir; her biri
  kendi aralığı dışında uyarır.
- Girdap kopmasının başlangıcında GPU ve CPU lattice Boltzmann çözümleri
  1e-4 ölçütünün üstünde, 7.1e-4'e kadar (göreli L2) ayrışır; kilitlenmeden
  sonra yeniden uyuşur ([docs/lbm.md](docs/lbm.md)).
- Kare hızı ve iş hacmi değerleri tek bir makinede ölçülmüştür (AMD Radeon RX
  9060 XT); belirtimdeki GTX 1660 sınıfı hedefler o sınıf donanımda
  ölçülmemiştir.
- Kara delikler yalnız Schwarzschild'dir (dönme yok); yakınlarındaki dinamik
  genel görelilik değil, sözde-Newton potansiyeli kullanır.
- Giriş: sabit kütle, kaldırma yok, dönme yok, ablasyon yok; ısı akısı
  gösterim amaçlı bir durma noktası bağıntısıdır.
- J2000 Güneş sistemi, JPL Horizons durum vektörlerinden alınan on noktasal
  kütledir (asteroitler, basıklık ve görelilik yok); efemeristen yılda
  yaklaşık 1e-6 AU uzaklaşır.
- Aero çalışma alanı lattice Boltzmann çözücüsünü CPU üzerinde çalıştırır;
  büyük 3B ızgaralar orada yavaştır.
- İçe aktarılan modeller doğrulanmaz; indirilen modellerin lisansları
  denetlenmez (sorumluluk kullanıcıdadır).

## Depo düzeni

| Yol | İçerik |
|---|---|
| `src/core` | birimler, cisimler, kütleçekimi, integratörler, kara delikler, olaylar (grafik yok) |
| `src/geom` | primitifler, ağ içe aktarma ve onarım, vokselleştirme, panel ağları; URL indirmeleri için `geom/net` |
| `src/aero` | panel yöntemi, Newton teorisi, lattice Boltzmann, hesaplama arka ucu arayüzü |
| `src/atmo` | atmosfer modelleri ve giriş dinamiği köprüsü |
| `src/scene` | senaryo ve deney biçimleri |
| `src/render` | OpenGL 4.6 render, gölgelendiriciler, kullanıcı arayüzü bileşenleri, GPU hesaplama arka ucu |
| `src/app` | simülasyon iş parçacıkları, çalışma alanları, yapılandırma, `orrery_app` çalıştırılabilir dosyası |
| `tests` | birim, özellik, doğrulama (`tests/aero_validation`) ve altın görüntü testleri |
| `assets` | hazır senaryolar ve deneyler |
| `docs` | başvuru belgeleri ve mimari karar kayıtları (`docs/adr`) |
| `tools` | `check_deps.py` (bağımlılık sınırları), `check_docs.py` (belge bağlantıları ve fizik dizini), `make_screenshots.py` (README görüntüleri) |

Kütüphaneler yalnız tek yönde bağımlıdır: core, geom, atmo ve simülasyon
tarafı hiçbir zaman OpenGL, GLFW ya da Dear ImGui'ye bağlanmaz (CI'da
denetlenir).

## Belgeler

- [docs/README.md](docs/README.md) - tüm belgelerin ve karar kayıtlarının dizini
- [docs/physics.md](docs/physics.md) - formül dizini, kaynaklar, yaklaşıklıklar ve ölçülen sonuçlar
- [docs/app.md](docs/app.md) - uygulama döngüsü, zaman hızlandırma bütçesi, çalışma alanları, kısayollar, yapılandırma, hata yüzeyi
- [docs/rendering.md](docs/rendering.md) - render, kullanıcı arayüzü, altın görüntüler
- [docs/aero.md](docs/aero.md), [docs/lbm.md](docs/lbm.md) - aerodinamik çözücüler
- [docs/aero-workspace.md](docs/aero-workspace.md), [docs/aero-visualisation.md](docs/aero-visualisation.md) - Aero çalışma alanı ve görünümü
- [docs/atmosphere.md](docs/atmosphere.md), [docs/entry-workspace.md](docs/entry-workspace.md) - atmosferler, giriş ve Entry çalışma alanı
- [docs/geometry-import.md](docs/geometry-import.md), [docs/url-import-security.md](docs/url-import-security.md) - geometri içe aktarma ve indirme sınırları
- [docs/roadmap.md](docs/roadmap.md) - V1, V2 ve V3 yol haritası, ertelenen soyutlamalar ve bağlaşım deseni
- [docs/screenshots.md](docs/screenshots.md) - README ekran görüntülerinin yeniden üretilmesi
- [docs/adr](docs/adr/README.md) - mimari karar kayıtları

Belgeler İngilizcedir. Bu dosyanın İngilizce sürümü: [README.md](README.md).
