#! /usr/bin/env node

import rosnodejs from "rosnodejs";
import fetch from "node-fetch";

// Adapted from: https://gist.github.com/ornicar/a097406810939cf7be1df8ea30e94f3e
/* FOR NODEJS
Utility function to read a ND-JSON HTTP stream.
`processLine` is a function taking a JSON object. It will be called with each element of the stream.
`response` is the result of a `fetch` request.
See usage example in the next file.
*/
const PAT = "lip_T2E2rY0sd498e8DW559z";
const readStream = (processLine, onlyReadFirst) => response => {
    const matcher = /\r?\n/;
    const decoder = new TextDecoder();
    let buf = '';
    return new Promise((resolve, fail) => {
        response.body.on('data', v => {
            const chunk = decoder.decode(v, { stream: true });
            buf += chunk;

            const parts = buf.split(matcher);
            buf = parts.pop();
            for (const i of parts.filter(p => p)) processLine(JSON.parse(i));
            if (onlyReadFirst) {
                resolve();
            }
        });
        response.body.on('end', () => {
            if (buf.length > 0) processLine(JSON.parse(buf));
            resolve();
        });
        response.body.on('error', fail);
    });
};


async function request_event_stream(game_event_publisher, challenge_event_publisher) {
    const response = await fetch("https://lichess.org/api/stream/event", {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then(readStream((line) => {
        if (line.type === "gameStart" || line.type === "gameFinish") {
            game_event_publisher.publish(line);
        } else if (line.type === "challenge" ||
                   line.type === "challengeCanceled" ||
                   line.type === "challengeDeclined") {
            challenge_event_publisher.publish({
              type: line.type,
              challenge: {
                id: line.challenge.id,
                url: line.challenge.url,
                status: line.challenge.status,
                color: line.challenge.color,
                challenger: {
                  rating: line.challenge.challenger.rating,
                  provisional: line.challenge.challenger.provisional,
                  online: line.challenge.challenger.online,
                  name: line.challenge.challenger.name,
                  patron: line.challenge.challenger.patron,
                  id: line.challenge.challenger.id
                }
              }
            });
        }
    }, false))
    .catch((e) => {console.error(e);});
}

function create_player(playerObj) {
    const isAI = Object.keys(playerObj).includes("aiLevel");
    return {
      aiLevel: isAI ? playerObj.aiLevel : 0,
      id:      isAI ? ""   : playerObj.id,
      name:    isAI ? "AI" : playerObj.name,
      rating:  isAI ? 0    : playerObj.rating
    };
}

// type OptionalProperty<T>: { key: string, def: T }
// set_optional_properties(Object: object, Object: msg_object, OptionalProperty[]: optional_properties): Object
function set_optional_properties(object, msg_object, optional_properties) {
    const object_keys = Object.keys(object);
    optional_properties.map((key) => {
        const included = object_keys.includes(key.key);
        if (included) {
            msg_object[key.key] = object[key.key];
        } else {
            msg_object[key.key] = key.def;
        }
    });
}

async function request_move_stream(request, response) {
    const game_state_publisher = rosnodejs.nh.advertise("/move_stream", "lichess_api/GameState");
    // Need to call once to get the first stream element which contains information about which
    // color the user will be playing as.
    await fetch(`https://lichess.org/api/board/game/stream/${request.id}`, {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then(readStream((line) => {
        if (line.error !== undefined) {
            response.success = false;
            response.error = line.error;
            rosnodejs.log.error(`Failed to get move stream: ${json.error}`);
        }
        else if (line.type === "gameFull") {
            let white = create_player(line.white);
            let black = create_player(line.black);
            response.success = true;
            response.error = "";
            response.speed = line.speed;
            response.white = white;
            response.black = black;
            const properties = [
              {key: "winner",    def: ""},
              {key: "wdraw",     def: false},
              {key: "bdraw",     def: false},
              {key: "wtakeback", def: false},
              {key: "btakeback", def: false}
            ];
            response.initial_state = { ...line.state };
            set_optional_properties(line.state, response.initial_state, properties);
            delete response.initial_state.type;
        }
    }, true))
    .catch((e) => {
        console.error(e);
    });

    // This stream will only publish the moves when they occur
    fetch(`https://lichess.org/api/board/game/stream/${request.id}`, {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then(readStream((line) => {
        rosnodejs.log.info(JSON.stringify(line, null, 4));
        if (line.type === "gameState") {
            const properties = [
              {key: "winner",    def: ""},
              {key: "wdraw",     def: false},
              {key: "bdraw",     def: false},
              {key: "wtakeback", def: false},
              {key: "btakeback", def: false}
            ];
            set_optional_properties(line, line, properties);
            game_state_publisher.publish(line);
        }
    }, false))
    .catch((e) => {
        console.error(e);
    });

    return response;
}

async function challenge_ai(request, response) {
    await fetch("https://lichess.org/api/challenge/ai", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "Authorization": `Bearer ${PAT}`
        },
        body: `level=${request.level}&color=${request.color}`
    })
    .then((res) => res.json())
    .then((json) => {
        if (json.error !== undefined) {
            rosnodejs.log.error(`Failed to challenge AI: ${json.error}`);
            response.success = false;
            response.error = json.error;
            response.id = "";
        } else {
            response.success = true;
            response.error = "";
            response.id = json.id;
        }
    })
    .catch((e) => {
        console.error(e);
    });

    return response;
}

async function make_move(request, response) {
    await fetch(`https://lichess.org/api/board/game/${request.id}/move/${request.move}`, {
        method: "POST",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => res.json())
    .then((json) => {
        if (json.ok !== undefined) {
            response.success = true;
            response.error_msg = "";
        } else {
            rosnodejs.log.error(`Failed to make move: ${json.error}`);
            response.success = false;
            response.error = json.error;
        }
    })
    .catch((e) => {console.error(e);});
    return response;
}

async function abort_game(request, response) {
     await fetch(`https://lichess.org/api/board/game/${request.id}/abort`, {
        method: "POST",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => res.json())
    .then((json) => {
        if (json.ok !== undefined) {
            response.success = true;
            response.error_msg = "";
        } else {
            response.success = false;
            response.error = json.error;
            rosnodejs.log.error(`Failed to abort: ${json.error}`);
        }
    })
    .catch((e) => {console.error(e);});
    return response;
}

async function resign_game(request, response) {
     await fetch(`https://lichess.org/api/board/game/${request.id}/resign`, {
        method: "POST",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => res.json())
    .then((json) => {
        if (json.ok !== undefined) {
            response.success = true;
            response.error_msg = "";
        } else {
            response.success = false;
            response.error = json.error;
            rosnodejs.log.error(`Failed to resign: ${json.error}`);
        }
    })
    .catch((e) => {console.error(e);});
    return response;
}

rosnodejs.initNode("/lichess_api").then(() => {
    rosnodejs.log.info("node started");
    const nh = rosnodejs.nh;
    request_event_stream(
      nh.advertise("/game_event_stream", "lichess_api/GameEvent"),
      nh.advertise("/challenge_event_stream", "lichess_api/ChallengeEvent")
    );
    nh.advertiseService("/challenge_ai", "lichess_api/ChallengeAI", challenge_ai);
    nh.advertiseService("/stream_game", "lichess_api/StreamGameState", request_move_stream);
    nh.advertiseService("/make_move", "lichess_api/MakeMove", make_move);
    nh.advertiseService("/abort", "lichess_api/Abort", abort_game);
    nh.advertiseService("/resign", "lichess_api/Resign", resign_game);
});
