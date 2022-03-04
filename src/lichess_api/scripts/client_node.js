#! /usr/bin/env node

import rosnodejs from "rosnodejs";

rosnodejs.initNode("/js_node_client").then(() => {
    let id = "";
    let game_state_subscriber;
    const nh = rosnodejs.nh;
    const echo = (msg) => { console.debug(msg); };
    const game_event_subscriber = nh.subscribe("/game_event_stream", "lichess_api/GameEvent", (msg) => {
        console.debug("from game event stream");
        console.debug(msg);
    });

    const stream_client = nh.serviceClient("/stream_game", "lichess_api/StreamGameState");
    const challenge_client = nh.serviceClient("/challenge_ai", "lichess_api/ChallengeAI");

    challenge_client.call({ level: 1, color: "black" })
    .then((res) => {
        console.debug(res);
        stream_client.call({ id: res.id })
        .then((res) => {
            console.debug(res);
            game_state_subscriber = nh.subscribe("/move_stream", "lichess_api/GameState", (msg) => {
                console.debug("from move stream");
                console.debug(msg);
            });
        });
    })
    .catch((e) => {console.error(e);});
});

