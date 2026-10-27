//! cpguard 절차 매크로.
//!
//! - `#[obfuscate]` : 함수/메서드 본문을 **FSM 디스패치 루프**(부분 평탄화) + 불투명 술어 +
//!   가짜 분기로 감싼다. 의미는 보존하면서(본문을 정확히 1회 실행) 디컴파일 결과를 상태 기계로
//!   바꿔 읽기 어렵게 만든다. 자유 함수와 impl 메서드 모두 지원.
//! - `obf_str!("..")` : 문자열 리터럴을 컴파일타임에 XOR 암호화(런타임에 `String` 복호).

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ImplItemFn, ItemFn, LitStr};

/// 원본 블록을 의미 보존 FSM 디스패치 루프로 감싼다.
///
/// 상태 0→1→2 로 진행하다 상태 2 에서 본문을 **정확히 1회** 실행하고 그 값을 반환한다.
/// `opaque_*` 는 최적화기가 접지 못하는 불투명 함수라, 디컴파일러에는 진짜 상태 기계처럼 남는다.
fn wrap_block(block: &syn::Block) -> proc_macro2::TokenStream {
    quote! {
        {
            let mut __cpg_s: u32 = ::cpguard::opaque_seed();
            let __cpg_r = '__cpguard_fsm: loop {
                match __cpg_s {
                    0 => { __cpg_s = ::cpguard::opaque_next(__cpg_s, 1); }
                    1 => { __cpg_s = ::cpguard::opaque_next(__cpg_s, 2); }
                    2 => {
                        if ::cpguard::opaque_true() {
                            break '__cpguard_fsm #block;
                        } else {
                            ::cpguard::never()
                        }
                    }
                    _ => ::cpguard::never(),
                }
            };
            __cpg_r
        }
    }
}

/// 함수/메서드에 제어흐름 난독화를 적용한다.
///
/// ```ignore
/// #[cpguard::obfuscate]
/// fn critical(x: u64) -> u64 { x.wrapping_mul(2654435761) }
///
/// impl Foo {
///     #[cpguard::obfuscate]
///     fn method(&self, x: u64) -> u64 { x ^ self.k }
/// }
/// ```
#[proc_macro_attribute]
pub fn obfuscate(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // 자유 함수.
    if let Ok(f) = syn::parse::<ItemFn>(item.clone()) {
        let attrs = &f.attrs;
        let vis = &f.vis;
        let sig = &f.sig;
        let body = wrap_block(&f.block);
        return quote! { #(#attrs)* #vis #sig #body }.into();
    }
    // impl 메서드.
    match syn::parse::<ImplItemFn>(item) {
        Ok(f) => {
            let attrs = &f.attrs;
            let vis = &f.vis;
            let defaultness = &f.defaultness;
            let sig = &f.sig;
            let body = wrap_block(&f.block);
            quote! { #(#attrs)* #vis #defaultness #sig #body }.into()
        }
        Err(e) => e.to_compile_error().into(),
    }
}

/// 문자열 리터럴을 컴파일타임 XOR 암호화한다. 런타임에 `String` 으로 복호화된다.
#[proc_macro]
pub fn obf_str(input: TokenStream) -> TokenStream {
    let lit = parse_macro_input!(input as LitStr);
    let bytes = lit.value().into_bytes();

    let mut key: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in &bytes {
        key = (key ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3);
    }

    let enc: Vec<u8> = bytes
        .iter()
        .enumerate()
        .map(|(i, &b)| b ^ ((key >> ((i % 8) * 8)) as u8) ^ (i as u8).wrapping_mul(31))
        .collect();

    let enc_lits = enc.iter().map(|b| quote! { #b });
    quote! { ::cpguard::deobf(&[#(#enc_lits),*], #key) }.into()
}
