#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "parson.h"

#define FILE_PRE "file_"
#define FILE_SUF ".json"

int main(int argc, char **argv) {

    char* line;
    ssize_t length;
    FILE* file = fopen("/proc/meminfo", "r");

    // length = getline

    // char buffer[50];
    // char num_buffer[50];
    // uint8_t* num;

    // for (int i = 0; i < 4; i++) {
    //     memset(buffer, 0, 50);
    //     memset(num_buffer, 0, 50);
    //     sprintf(num_buffer, "%d", i);
    //     strcat(buffer, FILE_PRE);
    //     strcat(buffer, num_buffer);
    //     strcat(buffer, FILE_SUF);

    //     FILE* file = fopen(buffer, "w");

    //     JSON_Value *root_value = json_value_init_object();
    //     JSON_Object *root_object = json_value_get_object(root_value);
    //     char *serialized_string = NULL;
    //     json_object_set_string(root_object, "name", "John Smith");
    //     json_object_set_number(root_object, "age", i);
    //     json_object_dotset_string(root_object, "address.city", "Cupertino");
    //     json_object_dotset_value(root_object, "contact.emails", json_parse_string("[\"email@example.com\",\"email2@example.com\"]"));
    //     serialized_string = json_serialize_to_string(root_value);
    //     fputs(serialized_string, file);

    //     json_free_serialized_string(serialized_string);
    //     json_value_free(root_value);

    //     fclose(file);
    // }

    // printf("Check those files cause Im about to delete em: ");

    // scanf("%d", &num);

    // for (int i = 0; i < 4; i++) {
    //     memset(buffer, 0, 50);
    //     memset(num_buffer, 0, 50);
    //     sprintf(num_buffer, "%d", i);
    //     strcat(buffer, FILE_PRE);
    //     strcat(buffer, num_buffer);
    //     strcat(buffer, FILE_SUF);
    //     remove(buffer);
    // }

    return 0;
}