# Autonomous Parking with Nonholonomic Motion Planning

Otonom Sürüş ve Optimal Kontrol kapsamında, nonholonomic kısıtlı bir aracın dar park alanlarında çarpışmasız planlanması ve fiziksel araç modeliyle izlenmesi projesi.

## 1. Proje Amacı ve Problem Tanımı

**Amaç:** Otonom bir aracın, park etmiş araçlar ve dar manevra alanları arasında kendisine ayrılan park boşluğuna güvenli biçimde yerleşmesini sağlayan bir planlama ve kontrol zinciri geliştirmek ve sanal bir park alanında göstermektir.

**Problem:** Araç yanal hareket edemez ve minimum dönüş yarıçapına sahiptir; bu yüzden düz bir geometrik yol her zaman uygulanabilir değildir. Dar yerlerde ileri-geri manevralar gerekir. Geometrik olarak mümkün bir yol ile araç denetimi tarafından izlenebilir bir yol arasındaki fark bu çalışmada açıkça gösterilecektir.

Kinematik bisiklet modeli (arka aks referanslı), `L` aks mesafesi, `δ` direksiyon açısı olmak üzere:

```
ẋ = v·cos θ,   ẏ = v·sin θ,   θ̇ = (v/L)·tan δ
|δ| ≤ δ_max  →  ρ_min = L / tan(δ_max)
```

Nonholonomic kısıt: `ẋ·sin θ − ẏ·cos θ = 0`

**Görev:** `x_start` pozundan `x_goal` pozuna, araç gövdesi engellerle kesişmeden ve giriş sınırları içinde kalarak bir yörünge bulmak ve bunu izlemek.

**Araştırma soruları:**
- Geometrik olarak mümkün bir yol hangi koşullarda araç tarafından izlenemez?
- Planlayıcı ile izleme denetleyicisinin etkileşimi başarıyı nasıl etkiler?
- Park alanı geometrisi değiştikçe manevralar ve başarı nasıl değişir?

## 2. Kapsam ve İş Paketleri

**Kapsam içi:** 2B sanal park alanı, statik engeller, kinematik/dinamik araç modeli, ileri-geri manevralar, planlayıcı ve denetleyici karşılaştırması, basit gürültü ve model hatası.

**Kapsam dışı:** Gerçek araç testi, algılama ve yerelleştirme geliştirme, hareketli engeller.

| İP | Başlık |
|----|--------|
| İP1 | Literatür taraması ve problem formülasyonu |
| İP2 | Araç modeli ve simülasyon ortamı |
| İP3 | Planlayıcıların gerçeklenmesi |
| İP4 | İzleme denetleyicilerinin gerçeklenmesi |
| İP5 | Planlayıcı–denetleyici entegrasyonu |
| İP6 | Deney tasarımı ve ölçümler |
| İP7 | Analiz ve raporlama |

## 3. Kullanılan Yöntemler ve Algoritmalar

**Configuration Space:** Araç `C = ℝ² × S¹` uzayında `(x, y, θ)` ile temsil edilir; engeller bu uzaya taşınarak çarpışma kontrolü yapılır.

**Planlama yaklaşımları:**

| Yaklaşım | Güçlü yön | Zayıf yön |
|----------|-----------|-----------|
| Hybrid A\* (+ Reeds–Shepp) | Kinematik olarak uygulanabilir yol, hızlı | Optimal değil, çözünürlüğe duyarlı |
| State Lattice / Motion Primitives | Tekrarlanabilir, kontrollü ilkel kümesi | Dar alanda ilkel çeşitliliği sınırlı |
| Sampling-Based (RRT / RRT\*) | Karmaşık geometride esnek | Dar geçitte yavaş, yol pürüzlü |
| Trajectory Optimization | Pürüzsüz, güvenlik payı ayarlanabilir | Yerel minimum, iyi başlangıç ister |

**İzleme denetleyicileri:** Pure Pursuit, Stanley, LQR / MPC. Cusp noktalarında yön değişimi segment bazlı izleme ile yönetilir.

**Planlayıcı–denetleyici etkileşimi:** Plan; konum, yön, hız ve yön işaretini içeren bir yörünge olarak izleyiciye verilir. İzleme hatası eşiği aşarsa yeniden planlama tetiklenir.

**Çarpışma kontrolü:** Araç gövdesi dikdörtgen/çoklu daire ile modellenir, güvenlik payı parametreleştirilir.

## 4. Simülasyon Ortamı

- **Yazılım:** `[dil, kütüphaneler, görselleştirme aracı]`
- **Araç parametreleri:** `L = [ ]`, `δ_max = [ ]`, boyutlar `[ ]`, maks. hız `[ ]`, `dt = [ ]`
- **Senaryolar:** dik park, paralel park, açılı park, dar koridor ve boşluğun yetersiz olduğu sınır durumlar. Boşluk genişliği, hedef yönelimi ve engel yerleşimi parametrik olarak değiştirilebilir.
- **Belirsizlik:** ölçüm gürültüsü, aktüatör gecikmesi, model parametre hatası.

## 5. Mimari

```
Ortam + Engeller ──▶ Planlayıcı ──▶ Yörünge ──▶ İzleme Denetleyicisi
                          ▲                              │
                          │ yeniden planla               ▼
                          └────────── Araç Modeli ◀── (v, δ) komutu
                                   (durum geri beslemesi)
```

## 6. Deneyler

**Metrikler:**

| Kategori | Metrik |
|----------|--------|
| Başarı | Çarpışmasız hedefe ulaşma oranı |
| Hedef doğruluğu | Son konum hatası (m), yönelim hatası (°) |
| Manevra karmaşıklığı | Cusp sayısı, yol uzunluğu |
| Güvenlik payı | Yürütme boyunca minimum engel mesafesi |
| İzlenebilirlik | Çapraz iz hatası, direksiyon/hız doyma oranı |
| Hesaplama | Planlama süresi, düğüm sayısı |

**Deney planı:**

| Deney | Amaç |
|-------|------|
| D1 | Planlayıcıların karşılaştırılması |
| D2 | Denetleyicilerin karşılaştırılması (aynı plan üzerinde) |
| D3 | Geometrik yol ile kinematik olarak izlenebilir yol farkı |
| D4 | Park yeri geometrisi ve boşluk genişliği duyarlılığı |
| D5 | Gürültü, gecikme ve model hatasının etkisi |
| D6 | Hesaplama maliyeti ve çözümsüz düzenlerin analizi |

Her deney farklı rastgele tohumlarla tekrarlanır; ortalama ± standart sapma verilir.

## 7. Sonuçlar

> Deney çıktılarıyla doldurulacaktır.

| Planlayıcı | Başarı (%) | Planlama süresi (s) | Cusp sayısı | Min. mesafe (m) |
|------------|-----------:|--------------------:|------------:|----------------:|
| Hybrid A\* | | | | |
| State Lattice | | | | |
| RRT\* | | | | |
| Trajectory Opt. | | | | |

| Denetleyici | Ort. çapraz iz hatası (m) | Son konum hatası (m) | Son yönelim hatası (°) |
|-------------|--------------------------:|---------------------:|-----------------------:|
| Pure Pursuit | | | |
| Stanley | | | |
| LQR / MPC | | | |

**Tartışma:** `[geometrik uygulanabilirlik vs. izlenebilirlik, ortam geometrisinin etkisi, plan geçersizleşmesi, sınırlılıklar]`

## 8. Kaynaklar

1. S. M. LaValle, *Planning Algorithms*, Cambridge University Press, 2006.
2. D. Dolgov et al., "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments," *IJRR*, 29(5), 2010.
3. J. A. Reeds, L. A. Shepp, "Optimal paths for a car that goes both forwards and backwards," *Pacific J. Math.*, 145(2), 1990.
4. L. E. Dubins, "On curves of minimal length with a constraint on average curvature," *American J. Math.*, 79(3), 1957.
5. M. Pivtoraiko, R. A. Knepper, A. Kelly, "Differentially Constrained Mobile Robot Motion Planning in State Lattices," *J. Field Robotics*, 26(3), 2009.
6. S. Karaman, E. Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning," *IJRR*, 30(7), 2011.
7. B. Paden et al., "A Survey of Motion Planning and Control Techniques for Self-Driving Urban Vehicles," *IEEE T-IV*, 1(1), 2016.
8. J. Kong et al., "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design," *IEEE IV Symposium*, 2015.
9. R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992.
10. G. M. Hoffmann et al., "Autonomous Automobile Trajectory Tracking for Off-Road Driving," *ACC*, 2007.

## Proje Üyeleri

| Ad Soyad | Öğrenci No |
|----------|-----------|
| Semih Özdemir | 032390072 |
| Muhsın Yılmaz | 032390035 |
| Mustafa Ozan Aydın | 032390049 |
| Eren Boylu | 032390068 |
| Arda Berat Kosor | 032390048 |
