fn main() {
    cc::Build::new()
        .file("src/fenv.c")
        .compile("fenv");
}
