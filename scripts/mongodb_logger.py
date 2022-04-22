from pymongo import MongoClient


class MongoDBLogger:
    def __init__(self, pilot=False):
        self.client = MongoClient('localhost', 27017)
        print("Connected to DB!")

        self.db = self.client["user_data"]
        self.col = self.db["pilot_trials"] if pilot else self.db["trials"]

    def add_task_to_db(self, participant_id, object_type, condition, transitions, success):
        """
        Add task to database
        :param participant_id: participant ID
        :param object_type: object type (i.e. lobster)
        :param condition: string representing condition (can be "gu", "gc", "tu", "tc")
        :param transitions: dict with fields `timestamp`, `button`, `current_state`, `next_state`
        :return:
        """

        # make new participant if it doesn't exist
        boneless_entry = {"_id": participant_id, "trials": []}
        self.col.update(
            {"_id": participant_id},
            {"$setOnInsert": boneless_entry},
            upsert=True
        )

        # make new trial if it doesn't exist
        if self.col.find({"_id": participant_id, "trials.condition": condition}).count() == 0:
            self.col.update(
                {'_id': participant_id},
                {"$push": {"trials": {"condition": condition, "tasks": []}}},
            )

        # add new task
        self.col.update(
            {'_id': participant_id, 'trials.condition': condition},
            {"$push":
                {"trials.$.tasks":
                    {
                        "object": object_type,
                        "success": success,
                        "transitions": transitions
                    }
                }
            },
            upsert=True
        )

        # add new transition
        # print(col.find_one({'_id': 42069, 'trials.condition': 'gu'}, {"trials.tasks": 0}))
        # trial = col.update(
        #     {'_id': 42069, 'trials.condition': 'gu'},
        #     {"$push":
        #         {"trials.$.tasks.0.transitions":
        #             {
        #                 "object": "Robot B",
        #                 "transitions": []
        #             }
        #         }
        #     }
        # )
