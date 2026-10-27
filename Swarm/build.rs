fn main() {
    const PRODUCT_VERSION: &str = "1.0.3";

    if cfg!(target_os = "windows") {
        let mut res = winres::WindowsResource::new();
        res.set("FileVersion", env!("CARGO_PKG_VERSION"));
        res.set("ProductVersion", PRODUCT_VERSION);
        // 하드닝(C-8): .rsrc 버전 리소스에서 알고리즘/제품 힌트를 제거한다.
        // 기존값 "Swarm Core - CA Planet 2D irregular strip packing" 은 네스팅/
        // 스트립패킹 알고리즘 도메인을 그대로 노출했다. 중립적인 모듈명으로 교체.
        // (버전/저작권은 정당한 메타데이터라 유지.)
        res.set("FileDescription", "cpswcore");
        res.set("ProductName", "cpswcore");
        res.set("LegalCopyright", "CA Planet");
        res.compile().unwrap();
    }

    println!("cargo:rustc-env=PRODUCT_VERSION={PRODUCT_VERSION}");
}
