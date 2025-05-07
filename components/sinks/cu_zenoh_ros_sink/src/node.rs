use zenoh::config::ZenohId;

pub struct Node<'a> {
    pub domain_id: u32,
    pub zid: ZenohId,
    pub id: u32,
    pub namespace: &'a str,
    pub name: &'a str,
}
