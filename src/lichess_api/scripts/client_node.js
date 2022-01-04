#! /usr/bin/env node

import rosnodejs from "rosnodejs";

rosnodejs.initNode("/js_node_client").then(() => {
    const nh = rosnodejs.nh;
    const echo = (msg) => { console.debug(msg); };
    const game_event_subscriber = nh.subscribe("/game_event_stream", "lichess_api/GameEvent", (msg) => {
        console.debug("from game event stream");
        console.debug(msg);
    });
    const game_def_subscriber = nh.subscribe("/game_def", "lichess_api/GameDef", (msg) => {
        console.debug("from game def");
        console.debug(msg);
    });
    const game_state_subscriber = nh.subscribe("/move_stream", "lichess_api/GameState", (msg) => {
        console.debug("from move stream");
        console.debug(msg);
    });

    const client = nh.serviceClient("/challenge_ai_srv", "lichess_api/ChallengeAI");
    client.call({ level: 1 })
    .then((res) => {console.debug(res);})
    .catch((e) => {console.err(e);});
});

