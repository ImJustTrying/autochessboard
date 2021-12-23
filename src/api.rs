//extern crate serde;

use reqwest::{header, blocking};
//use std::collections::HashMap;
use serde::Deserialize;

const PAT: &str = "Bearer lip_T2E2rY0sd498e8DW559z";

#[derive(Deserialize, Debug)]
struct Email {
    email: String
}

#[derive(Deserialize, Debug)]
struct Event {

}

fn generate_non_data_sending_client() -> core::result::Result<blocking::Client, Box<dyn std::error::Error>> {
    let mut headers = header::HeaderMap::new();
    headers.insert(header::AUTHORIZATION, header::HeaderValue::from_static(PAT));
    let client = blocking::Client::builder().default_headers(headers).build()?;
    Ok(client)
}

fn generate_data_sending_client() -> core::result::Result<blocking::Client, Box<dyn std::error::Error>> {
    let mut headers = header::HeaderMap::new();
    headers.insert(header::AUTHORIZATION, header::HeaderValue::from_static(PAT));
    headers.insert(header::CONTENT_TYPE, header::HeaderValue::from_static("application/x-www-form-urlencoded"));

    let client = blocking::Client::builder().default_headers(headers).build()?;
    Ok(client)
}



pub fn email_request() -> core::result::Result<(), Box<dyn std::error::Error>> {
    let client = generate_non_data_sending_client()?;
    let resp: Email = client.get("https://lichess.org/api/account/email").send()?.json()?;
    println!("{:#?}", resp);
    Ok(())
}

pub fn ai_challenge_request() -> core::result::Result<(), Box<dyn std::error::Error>> {
    let client = generate_data_sending_client()?;
    let resp = client.post("https://lichess.org/api/challenge/ai")
    .body("level=1&days=1")
    .send()?;
    println!("{:#?}", resp);
    Ok(())
}

pub fn get_events() -> core::result::Result<(), Box<dyn std::error::Error>> {
    let client = generate_non_data_sending_client()?;
    let resp = client.get("https://lichess.org/api/stream/event").send()?.()?;
    println!("{:#?}", resp);
    Ok(())
}
