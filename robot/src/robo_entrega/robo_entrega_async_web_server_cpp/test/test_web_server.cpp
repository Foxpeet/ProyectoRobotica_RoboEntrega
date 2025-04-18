#include "robo_entrega_async_web_server_cpp/http_reply.hpp"
#include "robo_entrega_async_web_server_cpp/http_server.hpp"
#include "robo_entrega_async_web_server_cpp/websocket_connection.hpp"
#include "robo_entrega_async_web_server_cpp/websocket_request_handler.hpp"

#include <signal.h>

using namespace robo_entrega_async_web_server_cpp;

volatile bool should_shutdown = false;
void sig_handler(int signo)
{
    should_shutdown = true;
}

static void http_body_echo(const robo_entrega_async_web_server_cpp::HttpRequest& request,
                           robo_entrega_async_web_server_cpp::HttpConnectionPtr connection,
                           const std::string& body)
{
    HttpReply::builder(HttpReply::ok).write(connection);
    connection->write(body);
}

static bool http_path_echo(const robo_entrega_async_web_server_cpp::HttpRequest& request,
                           robo_entrega_async_web_server_cpp::HttpConnectionPtr connection,
                           const char* begin, const char* end)
{
    HttpReply::builder(HttpReply::ok).write(connection);
    connection->write(request.path);
    return true;
}

static bool http_query_echo(const robo_entrega_async_web_server_cpp::HttpRequest& request,
                            robo_entrega_async_web_server_cpp::HttpConnectionPtr connection,
                            const char* begin, const char* end)
{
    HttpReply::builder(HttpReply::ok).write(connection);
    for (std::map<std::string, std::string>::const_iterator itr =
             request.query_params.begin();
         itr != request.query_params.end(); ++itr)
    {
        connection->write(itr->first + "=" + itr->second + "\n");
    }
    return true;
}

class WebsocketEchoer
{
public:
    WebsocketEchoer(WebsocketConnectionPtr websocket) : websocket_(websocket) {}
    void operator()(const WebsocketMessage& message)
    {
        websocket_->sendMessage(message);
    }

private:
    WebsocketConnectionPtr websocket_;
};
WebsocketConnection::MessageHandler
websocket_echo(const HttpRequest& request, WebsocketConnectionPtr websocket)
{
    return WebsocketEchoer(websocket);
}

int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        return 1;
    }

    HttpRequestHandlerGroup handler_group(
        HttpReply::stock_reply(HttpReply::not_found));

    handler_group.addHandlerForPath("/response/ok",
                                    HttpReply::stock_reply(HttpReply::ok));
    handler_group.addHandlerForPath("/response/created",
                                    HttpReply::stock_reply(HttpReply::created));
    handler_group.addHandlerForPath(
        "/response/accepted", HttpReply::stock_reply(HttpReply::accepted));
    handler_group.addHandlerForPath(
        "/response/forbidden", HttpReply::stock_reply(HttpReply::forbidden));
    handler_group.addHandlerForPath(
        "/response/not_found", HttpReply::stock_reply(HttpReply::not_found));
    handler_group.addHandlerForPath(
        "/response/internal_server_error",
        HttpReply::stock_reply(HttpReply::internal_server_error));

    handler_group.addHandlerForPath(
        "/a_static_response",
        HttpReply::static_reply(HttpReply::ok, "text/example", "A RESPONSE"));
    handler_group.addHandlerForPath("/http_body_echo",
                                    HttpRequestBodyCollector(http_body_echo));
    handler_group.addHandlerForPath("/http_path_echo.*", http_path_echo);
    handler_group.addHandlerForPath("/http_query_echo", http_query_echo);

    handler_group.addHandlerForPath(
        "/websocket_echo", WebsocketHttpRequestHandler(websocket_echo));

    auto test_dir = std::string(ROBO_ENTREGA_ASYNC_WEB_SERVER_CPP_TEST_DIR);
    handler_group.addHandlerForPath(
        "/test_files/.+", HttpReply::from_filesystem(
                              HttpReply::ok, "/test_files/", test_dir, false));
    handler_group.addHandlerForPath(
        "/test_files_with_dir/.+",
        HttpReply::from_filesystem(HttpReply::ok, "/test_files_with_dir/",
                                   test_dir, true));
    handler_group.addHandlerForPath(
        "/test_file", HttpReply::from_file(HttpReply::ok, "text/html",
                                           test_dir + "/test.html"));

    HttpServer server("0.0.0.0", "9849", handler_group, 1);

    server.run();

    while (!should_shutdown)
    {
        sleep(1);
    }

    return (0);
}
