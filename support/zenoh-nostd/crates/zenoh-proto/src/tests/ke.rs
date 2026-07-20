use crate::{KeyexprError, keyexpr};

fn intersect(left: &str, right: &str) -> bool {
    let left = keyexpr::new(left).unwrap();
    let right = keyexpr::new(right).unwrap();

    left.intersects(right)
}

fn err(ke: &str, err: KeyexprError) -> bool {
    let ke = keyexpr::new(ke);
    matches!(ke, Err(e) if e == err)
}

fn ok(ke: &str) -> bool {
    keyexpr::new(ke).is_ok()
}

#[test]
fn keyexpr_intersect() {
    assert!(intersect("a", "a"));
    assert!(intersect("a/b", "a/b"));
    assert!(intersect("*", "abc"));
    assert!(intersect("*", "xxx"));
    assert!(intersect("ab$*", "abcd"));
    assert!(intersect("ab$*d", "abcd"));
    assert!(intersect("ab$*", "ab"));
    assert!(!intersect("ab/*", "ab"));
    assert!(intersect("a/*/c/*/e", "a/b/c/d/e"));
    assert!(intersect("a/$*b/c/$*d/e", "a/xb/c/xd/e"));
    assert!(!intersect("a/*/c/*/e", "a/c/e"));
    assert!(!intersect("a/*/c/*/e", "a/b/c/d/x/e"));
    assert!(!intersect("ab$*cd", "abxxcxxd"));
    assert!(intersect("ab$*cd", "abxxcxxcd"));
    assert!(!intersect("ab$*cd", "abxxcxxcdx"));
    assert!(intersect("**", "abc"));
    assert!(intersect("**", "a/b/c"));
    assert!(intersect("ab/**", "ab"));
    assert!(intersect("**/xyz", "a/b/xyz/d/e/f/xyz"));
    assert!(!intersect("**/xyz$*xyz", "a/b/xyz/d/e/f/xyz"));
    assert!(intersect("**/xyz$*xyz", "a/b/xyzdefxyz"));
    assert!(intersect("a/**/c/**/e", "a/b/b/b/c/d/d/d/e"));
    assert!(intersect("a/**/c/**/e", "a/c/e"));
    assert!(intersect("a/**/c/*/e/*", "a/b/b/b/c/d/d/c/d/e/f"));
    assert!(!intersect("a/**/c/*/e/*", "a/b/b/b/c/d/d/c/d/d/e/f"));
    assert!(!intersect("ab$*cd", "abxxcxxcdx"));
    assert!(intersect("x/abc", "x/abc"));
    assert!(!intersect("x/abc", "abc"));
    assert!(intersect("x/*", "x/abc"));
    assert!(!intersect("x/*", "abc"));
    assert!(!intersect("*", "x/abc"));
    assert!(intersect("x/*", "x/abc$*"));
    assert!(intersect("x/$*abc", "x/abc$*"));
    assert!(intersect("x/a$*", "x/abc$*"));
    assert!(intersect("x/a$*de", "x/abc$*de"));
    assert!(intersect("x/a$*d$*e", "x/a$*e"));
    assert!(intersect("x/a$*d$*e", "x/a$*c$*e"));
    assert!(intersect("x/a$*d$*e", "x/ade"));
    assert!(!intersect("x/c$*", "x/abc$*"));
    assert!(!intersect("x/$*d", "x/$*e"));

    assert!(intersect("@a", "@a"));
    assert!(!intersect("@a", "@ab"));
    assert!(!intersect("@a", "@a/b"));
    assert!(!intersect("@a", "@a/*"));
    assert!(!intersect("@a", "@a/*/**"));
    assert!(!intersect("@a", "@a$*/**"));
    assert!(intersect("@a", "@a/**"));
    assert!(!intersect("**/xyz$*xyz", "@a/b/xyzdefxyz"));
    assert!(intersect("@a/**/c/**/e", "@a/b/b/b/c/d/d/d/e"));
    assert!(!intersect("@a/**/c/**/e", "@a/@b/b/b/c/d/d/d/e"));
    assert!(intersect("@a/**/@c/**/e", "@a/b/b/b/@c/d/d/d/e"));
    assert!(intersect("@a/**/e", "@a/b/b/d/d/d/e"));
    assert!(intersect("@a/**/e", "@a/b/b/b/d/d/d/e"));
    assert!(intersect("@a/**/e", "@a/b/b/c/d/d/d/e"));
    assert!(!intersect("@a/**/e", "@a/b/b/@c/b/d/d/d/e"));
    assert!(!intersect("@a/*", "@a/@b"));
    assert!(!intersect("@a/**", "@a/@b"));
    assert!(intersect("@a/**/@b", "@a/@b"));
    assert!(intersect("@a/@b/**", "@a/@b"));
    assert!(intersect("@a/**/@c/**/@b", "@a/**/@c/@b"));
    assert!(intersect("@a/**/@c/**/@b", "@a/@c/**/@b"));
    assert!(intersect("@a/**/@c/@b", "@a/@c/**/@b"));
    assert!(!intersect("@a/**/@b", "@a/**/@c/**/@b"));
}

#[test]
fn keyexpr_validation() {
    assert!(err("", KeyexprError::EmptyChunk));
    assert!(err("/demo/example/test", KeyexprError::EmptyChunk));
    assert!(err("demo/example/test/", KeyexprError::EmptyChunk));
    assert!(err("demo/$*/test", KeyexprError::LoneDollarStar));
    assert!(err("demo/$*", KeyexprError::LoneDollarStar));
    assert!(err(
        "demo/**/*/test",
        KeyexprError::SingleStarAfterDoubleStar
    ));
    assert!(err(
        "demo/**/**/test",
        KeyexprError::DoubleStarAfterDoubleStar
    ));
    assert!(err("demo//test", KeyexprError::EmptyChunk));
    assert!(err("demo/exam*ple/test", KeyexprError::StarInChunk));
    assert!(err("demo/example$*$/test", KeyexprError::DollarAfterDollar));
    assert!(err(
        "demo/example$*$*/test",
        KeyexprError::DollarAfterDollar
    ));
    assert!(err("demo/example#/test", KeyexprError::SharpOrQMark));
    assert!(err("demo/example?/test", KeyexprError::SharpOrQMark));
    assert!(err("demo/$/test", KeyexprError::UnboundDollar));

    assert!(ok("demo/example/test"));
    assert!(ok("demo/*"));
    assert!(ok("demo/**"));
    assert!(ok("demo/*/*/test"));
    assert!(ok("demo/*/**/test"));
    assert!(ok("demo/example$*/test"));
    assert!(ok("demo/example$*-$*/test"));
    assert!(ok("demo/example$*"));
}
